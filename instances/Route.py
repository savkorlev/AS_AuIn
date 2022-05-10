from instances.Trucks import Vehicle


class RouteObject:
    def __init__(self, customer_list: list, vehicle: Vehicle):
        self.customer_list: list = customer_list
        self.vehicle: Vehicle = vehicle
        self.currently_feasible: bool = False
        self.current_cost: float = 0
