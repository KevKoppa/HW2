"""
Create functions which accepts X¹, X², X³, r¹, r², r³, a¹, b¹, a², b², as input, and each return
one of the 5 callbacks which constitute an IPOPT problem: 
1. eval_f
2. eval_g
3. eval_grad_f
4. eval_jac_g
5. eval_hess_lag

Xⁱ is the vehicle_state of vehicle i at the start of the trajectory (t=0)
rⁱ is the radius of the i-th vehicle.
(aⁱ, bⁱ) define a halfpsace representing one of the two lane boundaries. 

The purpose of this function is to construct functions which can quickly turn 
updated world information into planning problems that IPOPT can solve.
"""
function create_callback_generator(trajectory_length=40, timestep=0.2, R = Diagonal([0.1, 0.5]), max_vel=10.0)
    # Define symbolic variables for all inputs, as well as trajectory

    # TODO Modifiy from other branch to appropriately work on cirlce tracks.
    # See generate_trajectory.jl for usage.
    return nothing
    
    #return (; full_cost_fn, 
    #        full_cost_grad_fn, 
    #        full_constraint_fn, 
    #        full_constraint_jac_triplet, 
    #        full_lag_hess_triplet,
    #        constraints_lb,
    #        constraints_ub)
end

"""
Predict a dummy trajectory for other vehicles.
"""
function constant_velocity_prediction(X0, trajectory_length, timestep)
    X = X0
    U = zeros(2)
    states = []
    for k = 1:trajectory_length
        X = evolve_state(X, U, timestep)
        push!(states, X)
    end
    states
end

"""
The physics model used for motion planning purposes.
Returns X[k] when inputs are X[k-1] and U[k]. 
Uses a slightly different vehicle model than presented in class for technical reasons.
"""
function evolve_state(X, U, Δ)
    V = X[3] + Δ * U[1] 
    θ = X[4] + Δ * U[2]
    X + Δ * [V*cos(θ), V*sin(θ), U[1], U[2]]
end

function lane_constraint(X, a, b, r)
    a'*(X[1:2] - a*r)-b
end

function collision_constraint(X1, X2, r1, r2)
    (X1[1:2]-X2[1:2])'*(X1[1:2]-X2[1:2]) - (r1+r2)^2
end

"""
Cost at each stage of the plan
"""
function stage_cost(X, U, R)
    cost = -0.1*X[3] + U'*R*U
end


"""
Assume z = [U[1];...;U[K];X[1];...;X[K]]
Return states = [X[1], X[2],..., X[K]], controls = [U[1],...,U[K]]
where K = trajectory_length
"""
function decompose_trajectory(z)
    K = Int(length(z) / 6)
    controls = [@view(z[(k-1)*2+1:k*2]) for k = 1:K]
    states = [@view(z[2K+(k-1)*4+1:2K+k*4]) for k = 1:K]
    return states, controls
end

function compose_trajectory(states, controls)
    K = length(states)
    z = [reduce(vcat, controls); reduce(vcat, states)]
end


