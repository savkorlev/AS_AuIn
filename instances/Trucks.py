class Vehicle:
    def __init__(self, type: str, city: str, city_cost_lvl: str, plateNr: str, fixed_cost_bool: bool, tax_ins_bool: bool):
        self.plateNr = plateNr
        self.type = type
        self.city = city
        self.max_duration = 600.0

        # set the city cost level according to the following dictionary
        city_tax_dict = {
            ('Paris', 'none'): 0,
            ('Paris', 'low'): 0.10,
            ('Paris', 'medium'): 0.25,
            ('Paris', 'high'): 0.40,
            ('Paris', 'ban'): 9999,
            ('NewYork', 'none'): 0,
            ('NewYork', 'low'): 0.10,
            ('NewYork', 'medium'): 0.25,
            ('NewYork', 'high'): 0.40,
            ('NewYork', 'ban'): 9999,
            ('Shanghai', 'none'): 0,
            ('Shanghai', 'low'): 0.05,
            ('Shanghai', 'medium'): 0.125,
            ('Shanghai', 'high'): 0.20,
            ('Shanghai', 'ban'): 9999,
        }
        city_tax = city_tax_dict.get((city, city_cost_lvl))

        if type == "MercedesBenzAtego":
            self.payload_kg = 2800
            self.payload_vol = 34.80 * 1000  # needs to be * 1000 because the volumes in nodes is given with unit m^3 * 10^-3
            self.range_km = 99999  # range for ICEVs is unlimited
            self.fixed_cost = 25.00 * fixed_cost_bool * (1 + 0.9085 * tax_ins_bool)  # 47.71 * fixed_cost_bool
            if city == "Paris":
                self.cost_km = 0.281
                self.cost_km_in = self.cost_km + city_tax  # only add city tax if vehicle is combustion
                self.cost_m = 0.383
            elif city == "NewYork":
                self.cost_km = 0.156
                self.cost_km_in = self.cost_km + city_tax
                self.cost_m = 0.429
            elif city == "Shanghai":
                self.cost_km = 0.178
                self.cost_km_in = self.cost_km + city_tax
                self.cost_m = 0.128

        elif type == "VWTransporter":
            self.payload_kg = 883
            self.payload_vol = 5.8 * 1000  # needs to be * 1000 because the volumes in nodes is given with unit m^3 * 10^-3
            self.range_km = 99999  # range for ICEVs is unlimited
            self.fixed_cost = 10.31 * fixed_cost_bool * (1 + 0.9085 * tax_ins_bool)  # 27.82 * fixed_cost_bool
            if city == "Paris":
                self.cost_km = 0.160
                self.cost_km_in = self.cost_km + city_tax
                self.cost_m = 0.369
            elif city == "NewYork":
                self.cost_km = 0.103
                self.cost_km_in = self.cost_km + city_tax
                self.cost_m = 0.459
            elif city == "Shanghai":
                self.cost_km = 0.113
                self.cost_km_in = self.cost_km + city_tax
                self.cost_m = 0.112

        elif type == "VWCaddy":
            self.payload_kg = 670
            self.payload_vol = 3.2 * 1000  # needs to be * 1000 because the volumes in nodes is given with unit m^3 * 10^-3
            self.range_km = 99999  # range for ICEVs is unlimited
            self.fixed_cost = 14.58 * fixed_cost_bool * (1 + 0.9085 * tax_ins_bool)  # 19.68 * fixed_cost_bool
            if city == "Paris":
                self.cost_km = 0.134
                self.cost_km_in = self.cost_km + city_tax
                self.cost_m = 0.369
            elif city == "NewYork":
                self.cost_km = 0.092
                self.cost_km_in = self.cost_km + city_tax
                self.cost_m = 0.459
            elif city == "Shanghai":
                self.cost_km = 0.099
                self.cost_km_in = self.cost_km + city_tax
                self.cost_m = 0.112

        elif type == "DeFuso":
            self.payload_kg = 2800
            self.payload_vol = 21.56 * 1000  # needs to be * 1000 because the volumes in nodes is given with unit m^3 * 10^-3
            self.range_km = 80  # range for BEV is limited
            self.fixed_cost = 48.21 * fixed_cost_bool
            if city == "Paris":
                self.cost_km = 0.102
                self.cost_km_in = self.cost_km  # no city tax for BEV
                self.cost_m = 0.383
            elif city == "NewYork":
                self.cost_km = 0.080
                self.cost_km_in = self.cost_km
                self.cost_m = 0.429
            elif city == "Shanghai":
                self.cost_km = 0.073
                self.cost_km_in = self.cost_km
                self.cost_m = 0.128

        elif type == "ScooterL":
            self.payload_kg = 905
            self.payload_vol = 7.67 * 1000  # needs to be * 1000 because the volumes in nodes is given with unit m^3 * 10^-3
            self.range_km = 205  # range for BEV is limited
            self.fixed_cost = 25.00 * fixed_cost_bool * (1 + 0.9085 * tax_ins_bool)  # 47.71 * fixed_cost_bool
            if city == "Paris":
                self.cost_km = 0.073
                self.cost_km_in = self.cost_km  # no city tax for BEV
                self.cost_m = 0.369
            elif city == "NewYork":
                self.cost_km = 0.068
                self.cost_km_in = self.cost_km
                self.cost_m = 0.459
            elif city == "Shanghai":
                self.cost_km = 0.066
                self.cost_km_in = self.cost_km
                self.cost_m = 0.112

        elif type == "ScooterS":
            self.payload_kg = 720
            self.payload_vol = 4.27 * 1000  # needs to be * 1000 because the volumes in nodes is given with unit m^3 * 10^-3
            self.range_km = 119  # range for BEV is limited
            self.fixed_cost = 19.77 * fixed_cost_bool * (1 + 0.9085 * tax_ins_bool)  # 37.74 * fixed_cost_bool
            if city == "Paris":
                self.cost_km = 0.070
                self.cost_km_in = self.cost_km  # no city tax for BEV
                self.cost_m = 0.369
            elif city == "NewYork":
                self.cost_km = 0.065
                self.cost_km_in = self.cost_km
                self.cost_m = 0.459
            elif city == "Shanghai":
                self.cost_km = 0.064
                self.cost_km_in = self.cost_km
                self.cost_m = 0.112

        elif type == "DouzeV2ECargoBike":
            self.payload_kg = 100
            self.payload_vol = 0.2 * 1000  # needs to be * 1000 because the volumes in nodes is given with unit m^3 * 10^-3
            self.range_km = 128
            self.fixed_cost = 2.92 * fixed_cost_bool * (1 + 0.2 * tax_ins_bool)  # 3.5 * fixed_cost_bool
            if city == "Paris":
                self.cost_km = 0.050
                self.cost_km_in = self.cost_km
                self.cost_m = 0.369
            elif city == "NewYork":
                self.cost_km = 0.049
                self.cost_km_in = self.cost_km
                self.cost_m = 0.459
            elif city == "Shanghai":
                self.cost_km = 0.049
                self.cost_km_in = self.cost_km
                self.cost_m = 0.112


def create_vehicles(city: str, city_cost_lv: str, num_Atego: int, num_VWTrans: int, num_VWCaddy: int, num_eFUSO: int,
                    num_eScooterWL: int, num_eScooterWS: int, num_eCargoBike: int,
                    fixed_cost_b: bool, tax_ins_b: bool) -> list[Vehicle]:
    listOfInitialVehicles = []

    for i in range(num_Atego):
        vehicleType = "MercedesBenzAtego"
        numberplate = "1MBA" + str(i + 1).zfill(3)
        listOfInitialVehicles.append(Vehicle(vehicleType, city, city_cost_lv, numberplate, fixed_cost_b, tax_ins_b))
    for i in range(num_VWTrans):
        vehicleType = "VWTransporter"
        numberplate = "2VWT" + str(i + 1).zfill(3)
        listOfInitialVehicles.append(Vehicle(vehicleType, city, city_cost_lv, numberplate, fixed_cost_b, tax_ins_b))
    for i in range(num_VWCaddy):
        vehicleType = "VWCaddy"
        numberplate = "3VWC" + str(i + 1).zfill(3)
        listOfInitialVehicles.append(Vehicle(vehicleType, city, city_cost_lv, numberplate, fixed_cost_b, tax_ins_b))
    for i in range(num_eFUSO):
        vehicleType = "DeFuso"
        numberplate = "4DeF" + str(i + 1).zfill(3)
        listOfInitialVehicles.append(Vehicle(vehicleType, city, city_cost_lv, numberplate, fixed_cost_b, tax_ins_b))
    for i in range(num_eScooterWL):
        vehicleType = "ScooterL"
        numberplate = "5SSL" + str(i + 1).zfill(3)
        listOfInitialVehicles.append(Vehicle(vehicleType, city, city_cost_lv, numberplate, fixed_cost_b, tax_ins_b))
    for i in range(num_eScooterWS):
        vehicleType = "ScooterS"
        numberplate = "6SSS" + str(i + 1).zfill(3)
        listOfInitialVehicles.append(Vehicle(vehicleType, city, city_cost_lv, numberplate, fixed_cost_b, tax_ins_b))
    for i in range(num_eCargoBike):
        vehicleType = "DouzeV2ECargoBike"
        numberplate = "7ECB" + str(i + 1).zfill(3)
        listOfInitialVehicles.append(Vehicle(vehicleType, city, city_cost_lv, numberplate, fixed_cost_b, tax_ins_b))

    return listOfInitialVehicles
