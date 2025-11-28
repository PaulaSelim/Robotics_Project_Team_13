import sympy as sp
import numpy as np
import sys

# --- Part 1: Symbolic Calculation (Run Once) ---

def symbolic_kinematics_calculation():
    """
    Performs symbolic calculations for position, velocity (Geometric Jacobian),
    and acceleration bias term (J_dot * q_dot) for a 4-DOF revolute manipulator.
    """

    print("Calculating all symbolic kinematics (Position, Geometric Jacobian, J_dot)...")
    print("THIS MAY TAKE SOME TIME. Be patient...")

    # --- Define Symbolic Variables ---
    t = sp.symbols('t')

    # Joint angles as functions of time (4 DOF)
    q1_t, q2_t, q3_t, q4_t = [sp.Function(f'q{i}')(t) for i in range(1, 5)]

    # Joint velocities (time derivatives)
    q1_dot_t, q2_dot_t, q3_dot_t, q4_dot_t = [sp.diff(q_t, t) for q_t in [q1_t, q2_t, q3_t, q4_t]]

    # Joint accelerations (second derivatives)
    q1_ddot_t, q2_ddot_t, q3_ddot_t, q4_ddot_t = [sp.diff(q_dot_t, t) for q_dot_t in [q1_dot_t, q2_dot_t, q3_dot_t, q4_dot_t]]

    # Standalone symbols for numeric lambdify usage
    q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4')
    q1_dot, q2_dot, q3_dot, q4_dot = sp.symbols('q1_dot q2_dot q3_dot q4_dot')
    q1_ddot, q2_ddot, q3_ddot, q4_ddot = sp.symbols('q1_ddot q2_ddot q3_ddot q4_ddot')

    # Link and offset symbols (as requested)
    L1, L2, L3, L4 = sp.symbols('L1 L2 L3 L4')
    a1, a2, a3, a4 = sp.symbols('a1 a2 a3 a4')

    # --- Symbol mapping dictionaries ---
    map_q_t_to_q = {q1_t: q1, q2_t: q2, q3_t: q3, q4_t: q4}
    map_q_dot_t_to_q_dot = {q1_dot_t: q1_dot, q2_dot_t: q2_dot, q3_dot_t: q3_dot, q4_dot_t: q4_dot}
    map_q_ddot_t_to_q_ddot = {q1_ddot_t: q1_ddot, q2_ddot_t: q2_ddot, q3_ddot_t: q3_ddot, q4_ddot_t: q4_ddot}

    # Helper: DH transformation (theta, d, a, alpha)
    def transformation_func(theta, d, a, alpha):
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
            [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
            [0,              sp.sin(alpha),                sp.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    # --- Forward Position Kinematics ---
    print("Step 1/3: Calculating Forward Position Kinematics...")

    # DH tuples given as: Tij = (theta_i, d_i, a_i, alpha_i)
    # As requested:
    # T01 = (-pi/2 + q1 , L1, 0, -pi/2)
    # T12 = (-pi/2 + q2 , a2, L2, -pi)
    # T23 = (q3, a3, L3, -pi)
    # T34 = (q4, 0, L4, 0)

    T01 = transformation_func(-sp.pi/2 + q1_t, L1, 0, -sp.pi/2)
    T12 = transformation_func(-sp.pi/2 + q2_t, a2, L2, -sp.pi)
    T23 = transformation_func(q3_t, a3, L3, -sp.pi)
    T34 = transformation_func(q4_t, 0, L4, 0)

    # Cumulative transforms
    T02 = sp.simplify(T01 * T12)
    T03 = sp.simplify(T02 * T23)
    T04 = sp.simplify(T03 * T34)

    # End-effector position (origin of frame 4)
    X_symbolic_t = T04[0:3, 3]  # 3x1 vector

    # --- Geometric Jacobian (linear part) ---
    print("Step 2/3: Calculating Geometric Jacobian (J)...")

    Z0 = sp.Matrix([0, 0, 1])
    O0 = sp.Matrix([0, 0, 0])

    Z1 = T01[0:3, 2]
    O1 = T01[0:3, 3]

    Z2 = T02[0:3, 2]
    O2 = T02[0:3, 3]

    Z3 = T03[0:3, 2]
    O3 = T03[0:3, 3]

    ON = T04[0:3, 3]

    # All joints revolute -> Jv_i = Z_{i-1} x (ON - O_{i-1})
    Jv1 = Z0.cross(ON - O0)
    Jv2 = Z1.cross(ON - O1)
    Jv3 = Z2.cross(ON - O2)
    Jv4 = Z3.cross(ON - O3)

    # Linear Jacobian 3x4
    Jv_symbolic_t = sp.Matrix.hstack(Jv1, Jv2, Jv3, Jv4)
    J_symbolic_t = Jv_symbolic_t  # 3x4 (position only)

    # --- Acceleration bias (J_dot * q_dot) ---
    print("Step 3/3: Calculating Acceleration Bias Term (J_dot * q_dot)...")

    q_vec_t = sp.Matrix([q1_t, q2_t, q3_t, q4_t])
    q_dot_vec_t = sp.Matrix([q1_dot_t, q2_dot_t, q3_dot_t, q4_dot_t])

    # X_dot = J * q_dot
    X_dot_symbolic_t = J_symbolic_t * q_dot_vec_t

    # Differentiate X_dot wrt time: includes J_dot * q_dot + J * q_ddot
    X_ddot_symbolic_t = sp.diff(X_dot_symbolic_t, t)

    # Subtract J * q_ddot to isolate bias term
    J_q_ddot_term = J_symbolic_t * sp.Matrix([q1_ddot_t, q2_ddot_t, q3_ddot_t, q4_ddot_t])
    X_acc_bias_symbolic_t = sp.simplify(X_ddot_symbolic_t - J_q_ddot_term)  # equals J_dot * q_dot

    # --- Substitute to get standalone expressions (no time functions) ---
    print("Substituting symbols for function generation...")

    subs_map = {
        **map_q_ddot_t_to_q_ddot,
        **map_q_dot_t_to_q_dot,
        **map_q_t_to_q
    }

    X_symbolic = sp.simplify(X_symbolic_t.subs(subs_map))
    J_symbolic = sp.simplify(J_symbolic_t.subs(subs_map))
    X_acc_bias_symbolic = sp.simplify(X_acc_bias_symbolic_t.subs(subs_map))

    print("Symbolic calculation complete.")

    # --- Create lambdified numerical functions ---
    print("Generating optimized Python functions (lambdify)...")

    # Order of symbols for lambdify (consistent use across the script)
    all_pos_symbols = [q1, q2, q3, q4, L1, L2, L3, L4, a1, a2, a3, a4]
    all_vel_symbols = [*all_pos_symbols, q1_dot, q2_dot, q3_dot, q4_dot]

    X_func = sp.lambdify(all_pos_symbols, X_symbolic, 'numpy')               # returns 3x1
    J_func = sp.lambdify(all_pos_symbols, J_symbolic, 'numpy')               # returns 3x4
    X_acc_bias_func = sp.lambdify(all_vel_symbols, X_acc_bias_symbolic, 'numpy')  # returns 3x1

    print("Functions generated successfully.")
    return X_symbolic, J_symbolic, X_func, J_func, X_acc_bias_func, all_pos_symbols, all_vel_symbols

# --- Part 2: Main Interactive Kinematic Scenario ---

def run_kinematic_scenario(X_func, J_func, X_acc_bias_func, all_pos_symbols, all_vel_symbols):
    """
    Interactive scenario: FK tests, velocity FK, Newton-Raphson IK (position-only),
    inverse velocity kinematics, and inverse acceleration kinematics using the
    generated numerical functions. Works for 4 DOF robot.
    """

    # Default robot numeric parameters (update here as requested)
    L_values = [0.04355, 0.140, 0.1329, 0.0]           # [L1,L2,L3,L4]
    a_values = [0.0, -0.03786, -0.0387, 0.0]          # [a1,a2,a3,a4]

    print('\n--- Using Link Lengths (L1..L4): ---')
    print(L_values)
    print('--- Using link offsets a1..a4: ---')
    print(a_values)

    try:
        # ---------------------------
        # Interactive FK (4 DOF)
        # ---------------------------
        print('\n\n--- Interactive Forward Kinematics (4 DOF) ---')
        print("Enter 4 joint angles in degrees to compute end-effector position (or 'next' to continue).")

        while True:
            input_str = input("Enter 4 joint angles (deg) or 'next': ")
            if input_str.strip().lower() == 'next':
                break
            try:
                q_values_user_deg = [float(x) for x in input_str.split()]
                if len(q_values_user_deg) != 4:
                    print(f"Please provide exactly 4 angles; got {len(q_values_user_deg)}.")
                    continue
                q_values_user_rad = np.deg2rad(q_values_user_deg)
                pos_args = (*q_values_user_rad, *L_values, *a_values)
                X_numerical = np.array(X_func(*pos_args)).reshape(3, 1)
                print("End-effector Position (m):", np.round(X_numerical.flatten(), 6))
            except ValueError:
                print("Invalid numbers. Try again.")
            except Exception as e:
                print("Error:", e)

        # ---------------------------
        # Interactive Velocity FK
        # ---------------------------
        print('\n\n--- Interactive Forward Velocity Kinematics (4 DOF) ---')
        print("Enter 4 joint angles (deg) and 4 joint velocities (deg/s) to compute X_dot (or 'next' to continue).")

        while True:
            input_q = input("Enter 4 joint angles (deg) or 'next': ")
            if input_q.strip().lower() == 'next':
                break
            try:
                q_deg = [float(x) for x in input_q.split()]
                if len(q_deg) != 4:
                    print("Expected 4 angles.")
                    continue
                qdot_deg = [float(x) for x in input("Enter 4 joint velocities (deg/s): ").split()]
                if len(qdot_deg) != 4:
                    print("Expected 4 velocities.")
                    continue

                q_rad = np.deg2rad(q_deg)
                qdot_rad = np.deg2rad(qdot_deg).reshape(4, 1)
                pos_args = (*q_rad, *L_values, *a_values)
                J_numerical = np.array(J_func(*pos_args)).astype(float)  # 3x4
                X_dot_numerical = J_numerical @ qdot_rad
                print("End-effector linear velocity (m/s):", np.round(X_dot_numerical.flatten(), 6))
            except Exception as e:
                print("Error:", e)

        # ---------------------------
        # Inverse Position Kinematics (Newton-Raphson)
        # ---------------------------
        print('\n\n--- Inverse Position Kinematics (Newton-Raphson) ---')
        target_str = input('Enter target [x y z] position in meters (e.g., "0.1 0 0.2"): ')
        X_target = np.array([float(val) for val in target_str.split()]).reshape(3, 1)

        guess_str = input('Enter initial guess for 4 joint angles (deg) e.g. "0 0 0 0": ')
        q_current_rad = np.deg2rad([float(val) for val in guess_str.split()])

        max_iterations = 200
        tolerance = 1e-6
        damping_factor = 0.01

        for i in range(max_iterations):
            pos_args = (*q_current_rad, *L_values, *a_values)
            X_current = np.array(X_func(*pos_args)).reshape(3, 1)
            error = X_target - X_current
            if np.linalg.norm(error) < tolerance:
                print(f"Converged in {i} iterations.")
                break

            J_numerical = np.array(J_func(*pos_args)).astype(float)  # 3x4
            # Damped least squares pseudo-inverse
            J_T = J_numerical.T
            lambda_sq_I = (damping_factor**2) * np.eye(3)
            # compute pseudo-inverse as: J^T * (J*J^T + lambda^2 I)^-1
            try:
                pinv = J_T @ np.linalg.inv(J_numerical @ J_T + lambda_sq_I)  # 4x3
            except np.linalg.LinAlgError:
                pinv = np.linalg.pinv(J_numerical)

            delta_q = (pinv @ error).flatten()  # length 4
            q_current_rad = q_current_rad + delta_q

            if i == max_iterations - 1:
                print("Warning: IK did not converge within max iterations.")

        final_pos = np.array(X_func(*((*q_current_rad), *L_values, *a_values))).reshape(3, 1)
        final_error = np.linalg.norm(X_target - final_pos)

        print("\n--- IK Result ---")
        print("Target       :", np.round(X_target.flatten(), 6))
        print("Achieved     :", np.round(final_pos.flatten(), 6))
        print("Final error  :", final_error)
        q_solution_deg = np.rad2deg(q_current_rad)
        print("Solved q (deg):", np.round(q_solution_deg, 4))

        q_for_next_step = q_current_rad.copy()

        # ---------------------------
        # Inverse Velocity Kinematics
        # ---------------------------
        print('\n\n--- Inverse Velocity Kinematics (IVK) ---')
        vel_target_str = input('Enter target end-effector velocity [vx vy vz] (m/s), e.g., "0.1 0 0": ')
        X_dot_target = np.array([float(val) for val in vel_target_str.split()]).reshape(3, 1)

        pos_args = (*q_for_next_step, *L_values, *a_values)
        J_numerical = np.array(J_func(*pos_args)).astype(float)  # 3x4

        q_dot_solution_rad = np.linalg.pinv(J_numerical) @ X_dot_target  # shape 4x1
        q_dot_solution_deg = np.rad2deg(q_dot_solution_rad.flatten())

        print("Solved q_dot (deg/s):", np.round(q_dot_solution_deg, 4))
        q_dot_for_next_step = q_dot_solution_rad.flatten()

        # ---------------------------
        # Acceleration Kinematics (Inverse Accel)
        # ---------------------------
        print('\n\n--- Inverse Acceleration Kinematics ---')
        accel_target_str = input('Enter target end-effector acceleration [ax ay az] (m/s^2), e.g., "0 0 0": ')
        X_ddot_target = np.array([float(val) for val in accel_target_str.split()]).reshape(3, 1)

        # Get J and its pseudo-inverse
        J_numerical = np.array(J_func(*pos_args)).astype(float)
        J_pinv_numerical = np.linalg.pinv(J_numerical)

        # Compute bias term (J_dot * q_dot) numerically from the lambdified function
        vel_args = (*q_for_next_step, *L_values, *a_values, *q_dot_for_next_step)
        X_acc_bias_numerical = np.array(X_acc_bias_func(*vel_args)).reshape(3, 1)

        # Solve for q_ddot: q_ddot = J_pinv * (X_ddot_target - bias)
        q_ddot_solution_rad = (J_pinv_numerical @ (X_ddot_target - X_acc_bias_numerical)).flatten()
        q_ddot_solution_deg = np.rad2deg(q_ddot_solution_rad)

        print("Solved q_ddot (deg/s^2):", np.round(q_ddot_solution_deg, 4))

        # Forward accel verification
        X_ddot_verification = (J_numerical @ q_ddot_solution_rad.reshape(4, 1)) + X_acc_bias_numerical
        print("Verified X_ddot (m/s^2):", np.round(X_ddot_verification.flatten(), 6))
        print("Acceleration error norm:", np.linalg.norm(X_ddot_target - X_ddot_verification))

        print("\n--- Scenario finished ---")

    except Exception as e:
        print("An error occurred in the main scenario:", e)
        import traceback
        traceback.print_exc()


# --- Part 3: Execute script ---

if __name__ == "__main__":
    try:
        (X_sym, J_sym,
         X_func, J_func, X_acc_bias_func,
         all_pos_symbols, all_vel_symbols) = symbolic_kinematics_calculation()

        # Optional: print symbolic results (can be very large)
        print("\n--- Symbolic Forward Position (X) ---")
        sp.pprint(X_sym)
        print("\n--- Symbolic Jacobian (J) ---")
        sp.pprint(J_sym)

        # Run interactive scenario
        run_kinematic_scenario(X_func, J_func, X_acc_bias_func, all_pos_symbols, all_vel_symbols)

    except Exception as e:
        print("A critical error occurred during symbolic setup:", e)
        import traceback
        traceback.print_exc()
    except KeyboardInterrupt:
        print("Symbolic calculation interrupted by user. Exiting.")
        sys.exit()