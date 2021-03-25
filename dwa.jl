"""
print config values
"""
function print_cfg()
    println("Current config values:")
    for (k, v) in cfg
        print("|$k => $v|")
    end
end

"""
update kinematics model using the current:
state = [1, 2, π, 0.1, 0.1]
vel = [0.2, 0.2]
"""
function kinematics(state, vel)
    state[1] += vel[1] * cfg["∇t"] * cos(state[3])
    state[2] += vel[1] * cfg["∇t"] * sin(state[3])
    state[3] += vel[2] * cfg["∇t"]
    state[4] = vel[1]
    state[5] = vel[2]
    return state
end

"""
caclulate cost:
goal = floor(abs.(rand(1, 2)* 10))
obs = floor(abs.(rand(10, 2) * 10))
traj = floor(abs.(randn(5, 5)* 10))
"""
function total_cost(goal, obst, traj)
    pos = traj[:, 1:2]
    for p = 1:size(pos, 1)
        for o = 1:size(obst, 1)
            @views dist = norm(obst[o, :] .- pos[p, :])
            if dist <= cfg["radius"]
                return Inf  # obs_cost
            end
        end
    end
    goal_cost = cfg["g_gain"] * norm(goal - pos[end])
    vel_cost = cfg["s_gain"] * (cfg["max_v"] - traj[end, 4])
    return goal_cost + vel_cost
end

"""
trajectory calculation
state = [1 2 π 0.1 0.1]
vel = [1 2]
"""
function gen_traj(state, vel)
    state = copy(state)
    traj = copy(state)
    for _ in 0:cfg["∇t"]:cfg["sim_time"]
        state = kinematics(state, vel)
        traj = [traj; state]
    end
    return traj
end

"""
state = [1 2 π 0.1 0.1]
"""
function opt_window(state)
    v_ω = [cfg["min_v"]]
    v = cfg["∇v"] * cfg["∇t"]
    ω = cfg["∇ω"] * cfg["∇t"]
    window = [max(cfg["min_v"], state[4] - v)
            min(cfg["max_v"], state[4] + v)
            max(-cfg["max_ω"], state[5] - ω)
            min(cfg["max_ω"], state[5] + ω)]
    return window
end

"""
rank set of trajectories by comparing cost for each
state = [1 2 π 0.1 0.1]
vel = [1 π/90]
window = opt_window(state)
goal = floor.(abs.(rand(1, 2)* 10))
obst = floor.(abs.(rand(10, 2) * 10))
"""
function best_traj(state, vel, goal, obst)
    min_cost = 10000
    opt_vel = vel
    opt_vel[1] = 0.0
    opt_traj = copy(state)
    window = opt_window(state)
    for v in window[1]:cfg["v_res"]:window[2]
        for w in window[3]:cfg["ω_res"]:window[4]
            traj = gen_traj(state, [v w])
            cost = total_cost(goal, obst, traj)
            if cost < min_cost
                min_cost = cost
                opt_vel = [v w]
                opt_traj = traj
            end
        end
    end
    return min_cost, opt_vel, opt_traj
end
"""main method to plot the trajectory values
"""
function main(cfg)
    println("Started..>>")
    total_cost = 0
    iter = 0
    state = [0.0 0.0 π/8.0 0.0 0.0]
    traj = copy(state)
    goal = [10. 10.]
    obst = [[-1. -1]; [0. 2.];
            [5. 5.]; [5. 6.];
            [5. 9.]; [8. 9.];
            [7. 9]; [12. 12.]]
    vel = [0.0 0.0]
    for i in 1:1000
        cost, vel, ltraj = best_traj(state, vel, goal, obst)
        state = kinematics(state, vel)
        traj = [traj; state]
        total_cost += cost
        iter = i
        if norm(goal[1:2] - state[1:2]) <= 1.0
            print("yay!, reached goal.. 😄\n")
            break
        else
            continue
        end
    end
    println(repeat("=", 71))
    println("final state and goal: ", state)
    println("Total cost for ", iter," iterations is:", total_cost)
    println(repeat("=", 71))
    return total_cost
end


cfg = Dict("min_v" => -0.5,  # min velocity
"max_v"=> 1.0,  # max velocity
"max_ω"=>  40. * π/180,  # max rot-velocity
"∇v"=> 0.2,  # accel
"∇ω"=> 40. * π/180,   # rot-accel
"sim_time"=> 3.,   # simulation time in seconds
"∇t" => 0.1,   # sim time resolution
"v_res"=> 0.01,   # velocity resolution
"ω_res"=> 0.1 * π/180,  # rot-velocity resolution
"g_gain"=> 1.,
"o_gain"=> 1.,
"s_gain"=> 1.,
"radius"=> 1.)
@time main(cfg)
