Contains several test nodes for the ur_robot package.<br/>
These nodes are:<br/>
<br/>
==> test_logging (launch test_logging.launch).<br/>
    Sets the robot in freedrive mode. Starts data logging when enter is pressed.<br/>
    Stops data logging when ctrl+C is pressed. To visualize the logged data go<br/>
    to ur_test/matlab/ and run from matlab load_logged_data.m and then<br/>
    plot_logged_data.m<br/>
<br/>
==> run_urscript (launch run_urscript.launch).<br/>
    Loads a URscript file and executes it. The scripts are located in ur_test/ur_scripts.<br/>
    You can change the arg 'urScript_file' in run_urscript.launch to choose the ur script<br/>
    file you want to execute.<br/>
<br/>
==> simple_control (launch simple_control.launch).<br/>
    Sets the robot in freedrive mode. The user can register a start and end pose by moving the<br/>
    robot around and pressing enter. The robot moves with position control to the start pose.<br/>
    Then a very naive velocity controller is implemented to move the robot from the initial to<br/>
    the final pose.
