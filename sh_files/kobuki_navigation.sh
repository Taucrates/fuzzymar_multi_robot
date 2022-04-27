#!/bin/bash
############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Starts the navigation of kobukis."
   echo
   echo "kobuki_navigation_ports.sh [options]"
   echo "options:"
   echo "-h		show brief help"
   echo "-m		loads mission_name to set initial pose and parameters of the robot"
   echo "-d		determines if the decision type is possibilistic (False) or deterministic (True, default)"
   #echo "g     Print the GPL license notification."
   #echo "h     Print this Help."
   #echo "v     Verbose mode."
   #echo "V     Print software version and exit."
   echo
}

############################################################
############################################################
# Main program                                             #
############################################################
############################################################

# Set variables
Mission="default"
Deterministic="True"

############################################################
# Process the input options. Add options as needed.        #
############################################################
# Get the options
while getopts ":hm:d:" option; do
   case $option in
      h) # display Help
         Help
         exit;;
      m) # Enter a mission name
         Mission=$OPTARG;;
      d) # Enter a decision type
         Deterministic=$OPTARG;;
     \?) # Invalid option
         echo "Error: Invalid option"
         exit;;
   esac
done

roslaunch fuzzymar_multi_robot kobuki_navigation.launch using_ports:=True kobuki_id:=$KOBUKI_ID robot_name:=$KOBUKI_NAME mission_name:=$Mission deterministic:=$Deterministic
