import sympy as sp
import numpy as np
from sympy.matrices import Matrix
import control

########## Initialization of variables & provided  
# Define the symbolic variables
x1, x2, u = sp.symbols('x1 x2 u')

# Define the differential equations
x1_dot = -x1 + 2 * x1**3 + x2 + 4*u
x2_dot = -x1 - x2 + 2 * u
##################################################


def find_equilibrium_points():
    '''
    1. Substitute input(u) = 0 in both equation for finding equilibrium points 
    2. Equate x1_dot, x2_dot equal to zero for finding equilibrium points 
    3. solve the x1_dot, x2_dot equations for the unknown variables and save the value to the variable namely "equi_points"
    '''

    ###### WRITE YOUR CODE HERE ################
    global x1_dot, x2_dot
    
    x1_dot_eq = x1_dot.subs(u, 0)
    x2_dot_eq = x2_dot.subs(u, 0)

    equations = [sp.Eq(x1_dot_eq, 0), sp.Eq(x2_dot_eq, 0)]

    equi_points = sp.solve(equations, (x1, x2))
    ############################################

    return equi_points

equi_points = find_equilibrium_points()
print("Equilibrium Points:", equi_points)

def find_A_B_matrices(eq_points):
    '''
    1. Substitute every equilibrium points that you have already find in the find_equilibrium_points() function 
    2. After substituting the equilibrium points, Save the Jacobian matrices A and B as A_matrices, B_matrices  
    '''
    
    A_matrix = sp.Matrix([
        [sp.diff(x1_dot, x1), sp.diff(x1_dot, x2)],
        [sp.diff(x2_dot, x1), sp.diff(x2_dot, x2)]
    ])
    
    B_matrix = sp.Matrix([
        [sp.diff(x1_dot, u)],
        [sp.diff(x2_dot, u)]
    ])
    A_matrices, B_matrices = [], []
    
    ###### WRITE YOUR CODE HERE ################
    for point in eq_points:
        A_matrices.append(A_matrix.subs({x1: point[0], x2: point[1]}))
        B_matrices.append(B_matrix.subs({x1: point[0], x2: point[1]}))
    ############################################
    
    return A_matrices, B_matrices

# Example usage
A_matrices, B_matrices = find_A_B_matrices(equi_points)
print("A_matrices:", A_matrices)
print("B_matrices:", B_matrices)

def find_eigen_values(A_matrices):
    '''
    1.  Find the eigen values of all A_matrices (You can use the eigenvals() function of sympy) 
        and append it to the 'eigen_values' list
    2.  With the eigen values, determine whether the system is stable or not and
        append the string 'Stable' if system is stable, else append the string 'Unstable'
        to the 'stability' list 
    '''
    
    eigen_values = []
    stability = []

    ###### WRITE YOUR CODE HERE ################
    for A in A_matrices:
        eigen_vals = A.eigenvals()
        eigen_values.append(eigen_vals)
        
        # Determine stability
        if all(sp.re(val).evalf() < 0 for val in eigen_vals):
            stability.append('Stable')
        else:
            stability.append('Unstable')
    ############################################
    return eigen_values, stability
eigen_values, stability = find_eigen_values(A_matrices)
print("Eigenvalues:", eigen_values)
print("Stability:", stability)


def compute_lqr_gain(jacobians_A, jacobians_B):
    K = 0
    '''
    This function is use to compute the LQR gain matrix K
    1. Use the Jacobian A and B matrix at the equilibrium point (-1,1) and assign it to A and B respectively for computing LQR gain
    2.  Compute the LQR gain of the given system equation (You can use lqr() of control module)
    3. Take the A matrix corresponding to the Unstable Equilibrium point (-1,1) that you have already found for computing LQR gain.
    4. Assign the value of gain to the variable K
    '''
    # Define the Q and R matrices
    Q = np.eye(2)  # State weighting matrix
    R = np.array([1])  # Control weighting matrix

    ###### WRITE YOUR CODE HERE ################
    # Find the unstable equilibrium point
    eigen_values, _ = find_eigen_values(jacobians_A)
    
    unstable_index = -1
    for i, eig_vals in enumerate(eigen_values):
        if any(sp.re(val).evalf() > 0 for val in eig_vals):
            unstable_index = i
            break
    
    if unstable_index == -1:
        raise ValueError("No unstable equilibrium point found.")
    
    # Extract the A and B matrices corresponding to the unstable equilibrium point
    A = np.array(jacobians_A[unstable_index]).astype(np.float64)
    B = np.array(jacobians_B[unstable_index]).astype(np.float64)
    
    # Compute the LQR gain
    K, _, _ = control.lqr(A, B, Q, R)
    ############################################
    return K
K = compute_lqr_gain(A_matrices, B_matrices)
print("LQR Gain:", K)