# Advanced Seminar Automotive Industry

## Development environment
The software was tested in the PyCharm environment. 
You can download a free version at [https://www.jetbrains.com/pycharm/download/](https://www.jetbrains.com/pycharm/download/).
## Python
Make sure to have Python installed that is at least version 3.9.0.
## Running the software
The most convenient way to start the software is to run the "main.py" file that is included in the project in "\AS_AuIn\src\main.py".
Before that, however, make sure that the following conditions are fulfilled:
1. The working directory is set up correctly. You can check the current working directory by navigating to "Run > Edit Configurations > main > Working directory". To set the correct working directory, select the software folder you downloaded. Therefore, the correctly set working directory must end with "\AS_AuIn".
2. The script path is set up correctly. You can check the current script path by navigating to "Run > Edit Configurations > main > Script path". To set the correct script path, select the "main.py" file within the software folder you downloaded. Therefore, the correctly set script path must end with "\AS_AuIn\src\main.py".
3. All the required external packages are installed. These packages are: "pandas", "matplotlib" and "seaborn". To install a package, first navigate to "File > Settings > Project: main.py > Python Interpreter". Then, click the plus sign on the right. Finally, type the name of the package and click "Install Package".
## Choosing the inputs
The software provides an option to enter the input data. To change the inputs, first navigate to "\AS_STS_code\src" and open the "main.py" file. The following inputs can be changed:
1. Set a city. You can choose among all the three cities. To select a city change the 15th line of code accordingly. The names are: NewYork, Paris, Shanghai.
2. Set a fleet. You can choose among all the seven vehicle types. To select a specific number of vehicles of each type, change the lines 17-23 of code accordingly. The vehicle types are:
   1. numI_Atego      - the number of Mercedes Benz Atego vehicles.
   2. numI_VWTrans    - the number of VW Transporter vehicles.
   3. numI_VWCaddy    - the number of VW Caddy panel van vehicles.
   4. numI_DeFuso     - the number of Daimler FUSO eCanter vehicles.
   5. numI_ScooterL   - the number of StreetScooter WORK-L vehicles.
   6. numI_ScooterS   - the number of StreetScooter WORK vehicles.
   7. numI_eCargoBike - the number of Douze V2 E-Cargo Bike vehicles.
The software can work with heterogeneous fleet. To optimise the current fleet enter the actual number of the vehicles the company has. To optimise the fleet, so it fits the city the best (regardless of the current fleet), enter 99 for each vehicle type. 
3. Turn on/off fixed costs. This option sets fixed costs active for all vehicles. To activate the fixed costs change the 26th line of code accordingly. Set the value of the parameter to "True" if you want to activate the fixed costs and "False" otherwise.
4. Turn on/off taxes and insurance. This option makes fixed costs for all non-leased vehicles ~90% higher (20% for bike). To activate the fixed costs change the 26th line of code accordingly. Set the value of the parameter to "True" if you want to activate taxes and insurance and "False" otherwise.
5. Set the demand factors. The factors are:
   1. To set the kilogram factor change the 30th line of code accordingly. The higher the value of "kg_factor" - the more weight of goods is requested by the customers.
   2. To set the volume factor change the 32nd line of code accordingly. The higher the value of "vol_factor" - the more volume of goods is requested by the customers.
6. Set the city cost level. Changes the level of restrictions on vehicles driving across the city center. To set the level change the 34th line of code accordingly. The levels are: 'none', 'low', 'medium', 'high', 'ban'.
7. Set the number of iterations the algorithm can run. To set the number of iterations change the 39th line of code accordingly. The higher the value of "max_iterations" - the slower the algorithm is but the more precise the solution is.
## Data analysis
The software provides a feature of saving the results (and input parameters) of the algorithm runs in separate ".csv" files for further analysis. The results are saved to "\AS_STS_code" directory. To disable this feature, first navigate to "\AS_STS_code\src" and open the "main.py" file. Then find the 214th line of code and put a "#" sign on the first position.
