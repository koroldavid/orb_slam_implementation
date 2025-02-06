import numpy as np

def calculate_rotation_difference(R1, R2):
    # Calculate the inverse of R1
    R1_inv = np.linalg.inv(R1)
    
    # Calculate the difference: R_diff = R1_inv * R2
    R_diff = np.dot(R1_inv, R2)
    
    return R_diff

def calculate_R2(R1, R_diff):
    # Calculate R2 = R1 * R_diff
    R2 = np.dot(R1, R_diff)
    return R2

def calculate_t_diff(R1, R2, t1, t2):
    # Calculate the inverse of R1
    R1_inv = np.linalg.inv(R1)
    
    # Calculate the translation difference: t_diff = R1_inv * (t2 - t1)
    t_diff = np.dot(R1_inv, (t2 - t1))
    
    return t_diff
    
# Example: Define rotation matrices for both cameras
R_c1 = np.array([
    [0.00555786,  0.99997899, -0.00333593],
    [0.99994638, -0.00552847,  0.00875655],
    [0.00873793, -0.00338442, -0.9999561],
])  # Example rotation matrix for camera 1

R_c2 = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0], 
])  # Example rotation matrix for camera 2

t1 = np.array([0.00073847, 0.00003206, 0.0001082])  # Translation vector for camera 1
t2 = np.array([0.0, 0.0, 0.0])  # Translation vector for camera 2

# Calculate the relative rotation matrix R_c1_c2
R_diff = calculate_rotation_difference(R_c1, R_c2)
print("Relative Rotation Matrix R_diff:")
print(R_diff)

print("calculate_t_diff T_diff")
print(calculate_t_diff(R_c1, R_c2, t1, t2))

