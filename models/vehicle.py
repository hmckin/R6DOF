class Rocket:
    def __init__(self, mass, moment_of_inertia, center_of_mass, center_of_pressure,
                 drag_coefficient, lift_coefficient, reference_area):
        self.mass = mass
        self.moment_of_inertia = moment_of_inertia
        self.center_of_mass = center_of_mass
        self.center_of_pressure = center_of_pressure
        self.drag_coefficient = drag_coefficient
        self.lift_coefficient = lift_coefficient
        self.reference_area = reference_area

    # Example method for mass depletion
    def burn_fuel(self, mass_loss):
        self.mass -= mass_loss
        # Optionally update inertia, center of mass, etc.
