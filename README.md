# Advanced Seminar Automotive Industry

## Development environment
The software was tested in the PyCharm environment. 
You can download a free version at [https://www.jetbrains.com/pycharm/download/](https://www.jetbrains.com/pycharm/download/).
## Python
Make sure to have Python installed that is at version 3.8.0.
## CPLEX
Make sure to have CPLEX installed.
## Running the software
The most convenient way to start the software is to run the "main.py" file that is included in the project in "\AS_AuIn\src\main.py".
Before that, however, make sure that the following conditions are fulfilled:
1. The working directory is set up correctly. You can check the current working directory by navigating to "Run > Edit Configurations > main > Working directory". To set the correct working directory, select the software folder you downloaded. Therefore, the correctly set working directory must end with "\AS_AuIn".
2. The script path is set up correctly. You can check the current script path by navigating to "Run > Edit Configurations > main > Script path". To set the correct script path, select the "main.py" file within the software folder you downloaded. Therefore, the correctly set script path must end with "\AS_AuIn\src\main.py".
3. All the required external packages are installed. These packages are: "pandas", "matplotlib", "numpy" and "docplex". To install a package, first navigate to "File > Settings > Project: main.py > Python Interpreter". Then, click the plus sign on the right. Finally, type the name of the package and click "Install Package".
