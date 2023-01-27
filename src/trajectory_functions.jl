"""
Create a function which accepts VEHICLE_STATES, VEHICLE_SIZES, and LANES, as input, and returns
the 5 callbacks which constitute an IPOPT problem: 
1. eval_f
2. eval_g
3. eval_grad_f
4. eval_jac_g
5. eval_hess_lag

vehicle_states is a vector of vehicle states
e.g. vehicle_states = [X¹, X², X³], where Xⁱ is the initial
position of the i-th vehicle. X¹ is the state of the ego-vehicle, for which
we will generate a trajectory.

vehicle_sizes is a vector of vehicle radii, one for each vehicle. 
e.g. vehicle_sizes = [r¹, r², r³]

lanes is a vector of halfspaces representing any lane boundaries that should be satisfied.

The purpose of this function is to construct functions which can quickly turn 
updated world information into planning problems that IPOPT can solve.
"""
function create_callback_generator(trajectory_length=40, timestep=0.2, R = Diagonal([0.1, 0.5]))

    function eval_f(z, vehicle_states, vehicle_sizes, lanes)
        states, controls = decompose_trajectory(z, trajectory_length)
        cost = sum(stage_cost(x, u, R) for (x,u) in zip(states, controls))
    end

    function eval_g(z, g, vehicle_states, vehicle_sizes, lanes)
        
    end

    function callback_generator(vehicle_states, vehicle_sizes, lanes)
        eval_f(z) = eval_f(z, vehicle_states, vehicle_sizes, lanes)
        eval_g(z, g) = eval_g(z, g, vehicle_states, vehicle_sizes, lanes)
        eval_grad_f(z, grad) = eval_grad_f(z, grad, vehicle_states, vehicle_sizes, lanes)
        eval_g(z, g) = eval_g(z, g, vehicle_states, vehicle_sizes, lanes)
        eval_g(z, g) = eval_g(z, g, vehicle_states, vehicle_sizes, lanes)
        return (; eval_f, eval_g, eval_grad_f, eval_jac_g, eval_hess_lag)
    end
end

function constant_velocity_prediction(X0, trajectory_length, timestep)
    states = []
    
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


function stage_cost(X, U, R)
    cost = -X[3] + U'*R*U
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
