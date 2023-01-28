function simulate(;
        rng = MersenneTwister(1),
        sim_steps = 100, 
        timestep = 0.2, 
        traj_length = 10,
        R = Diagonal([0.1, 0.5]),
        lane_width = 10, 
        lane_length = 100, 
        num_vehicles = 5, 
        min_r = 2, 
        max_r = 4, 
        max_vel = 10)
    sim_records = []
    a¹ = [0; 1]; b¹ = -lane_width / 2.0
    a² = [0; -1]; b² = -lane_width / 2.0

    # vehicle 1 is EGO
    vehicles = [generate_random_vehicle(rng, lane_width, lane_length, min_r, max_r, max_vel),]
    while length(vehicles) < num_vehicles
        v = generate_random_vehicle(rng, lane_width, lane_length, min_r, max_r, max_vel)
        if any(collision_constraint(v.state, v2.state, v.r, v2.r) < 1e-4 for v2 in vehicles)
            continue
        else
            push!(vehicles, v)
        end
    end
    #vehicles = [generate_random_vehicle(rng, lane_width, lane_length, min_r, max_r, max_vel) for _ in 1:num_vehicles]
    callbacks = create_callback_generator(traj_length, timestep, R, max_vel)

    for t = 1:sim_steps
        ego = vehicles[1]
        dists = [Inf; [norm(v.state[1:2]-ego.state[1:2])-v.r-ego.r for v in vehicles[2:end]]]
        closest = partialsortperm(dists, 1:2)
        V2 = vehicles[closest[1]]
        V3 = vehicles[closest[2]]
        
        trajectory = generate_trajectory(ego, V2, V3, a¹, b¹, a², b², callbacks, traj_length, timestep)

        push!(sim_records, (; vehicles=copy(vehicles), trajectory))

        vehicles[1] = (; state = trajectory.states[1], r=vehicles[1].r)
        foreach(i->vehicles[i] = (; state = evolve_state(vehicles[i].state, zeros(2), timestep), vehicles[i].r), 2:num_vehicles)
    end
    visualize_simulation(sim_records, a¹, b¹, a², b², traj_length)
end


function generate_random_vehicle(rng, lane_width, lane_length, min_r, max_r, max_vel)
    r = rand(rng)*(max_r-min_r) + min_r
    p1 = rand(rng)*lane_length
    p2 = rand(rng)*(lane_width-2r)+(r-lane_width/2)
    v = rand()*max_vel/2 + max_vel
    v = 0
    θ = 0.0
    (; state=[p1, p2, v, θ], r)
end

function generate_trajectory(ego, V2, V3, a¹, b¹, a², b², callbacks, trajectory_length, timestep)
    X1 = ego.state
    X2 = V2.state
    X3 = V3.state
    r1 = ego.r
    r2 = V2.r
    r3 = V3.r
   
    # refine callbacks with current values of parameters / problem inputs
    wrapper_f = function(z)
        callbacks.full_cost_fn(z, X1, X2, X3, r1, r2, r3, a¹, b¹, a², b²)
    end
    wrapper_grad_f = function(z, grad)
        callbacks.full_cost_grad_fn(grad, z, X1, X2, X3, r1, r2, r3, a¹, b¹, a², b²)
    end
    wrapper_con = function(z, con)
        callbacks.full_constraint_fn(con, z, X1, X2, X3, r1, r2, r3, a¹, b¹, a², b²)
    end
    wrapper_con_jac = function(z, rows, cols, vals)
        if isnothing(vals)
            rows .= callbacks.full_constraint_jac_triplet.jac_rows
            cols .= callbacks.full_constraint_jac_triplet.jac_cols
        else
            callbacks.full_constraint_jac_triplet.full_constraint_jac_vals_fn(vals, z, X1, X2, X3, r1, r2, r3, a¹, b¹, a², b²)
        end
        nothing
    end
    wrapper_lag_hess = function(z, rows, cols, cost_scaling, λ, vals)
        if isnothing(vals)
            rows .= callbacks.full_lag_hess_triplet.hess_rows
            cols .= callbacks.full_lag_hess_triplet.hess_cols
        else
            callbacks.full_lag_hess_triplet.full_hess_vals_fn(vals, z, X1, X2, X3, r1, r2, r3, a¹, b¹, a², b², λ, cost_scaling)
        end
        nothing
    end

    n = trajectory_length*6
    m = length(callbacks.constraints_lb)
    # setup IPOPT problem
    prob = Ipopt.CreateIpoptProblem(
        n,
        fill(-Inf, n),
        fill(Inf, n),
        length(callbacks.constraints_lb),
        callbacks.constraints_lb,
        callbacks.constraints_ub,
        length(callbacks.full_constraint_jac_triplet.jac_rows),
        length(callbacks.full_lag_hess_triplet.hess_rows),
        wrapper_f,
        wrapper_con,
        wrapper_grad_f,
        wrapper_con_jac,
        wrapper_lag_hess
    )

    controls = repeat([zeros(2),], trajectory_length)
    states = constant_velocity_prediction(X1, trajectory_length, timestep)
    zinit = compose_trajectory(states, controls)
    prob.x = zinit

    Ipopt.AddIpoptIntOption(prob, "print_level", 0)
    #Ipopt.AddIpoptStrOption(prob, "derivative_test", "second-order");
    #Ipopt.AddIpoptStrOption(prob, "hessian_approximation", "limited-memory");
    status = Ipopt.IpoptSolve(prob)

    if status != 0
        @infiltrate
        @warn "MCP not cleanly solved. IPOPT status is $(status)."
    end
    states, controls = decompose_trajectory(prob.x)
    (; states, controls, status)
end

function visualize_simulation(sim_results, a1,b1,a2,b2, lane_length)
    return
end
    

