
class ImpedanceControl:
    def __init__(self, M, B, K):
        self.M = M  # Mass coefficient
        self.B = B  # Damping coefficient
        self.K = K  # Spring coefficient
        
        # Initial values
        self.a = 0.0  # Current acceleration
        self.v = 0.0  # Current velocity
        self.x = 0.0  # Current position

    def compute_force(self, a_d, v_d, x_d):
        """Compute the external force based on desired and current states."""
        tau = self.M * (a_d - self.a) + self.B * (v_d - self.v) + self.K * (x_d - self.x)
        return tau

    def update_state(self, a_new, v_new, x_new):
        """Update the current state of the system."""
        self.a = a_new
        self.v = v_new
        self.x = x_new

# Test
controller = ImpedanceControl(M=1.0, B=1.0, K=1.0)
tau = controller.compute_force(a_d=1.0, v_d=1.0, x_d=1.0)
tau

#impedance control algorithm
