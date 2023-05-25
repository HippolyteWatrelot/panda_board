#!/bin/bash                                                                                                                                                                

mapfile BUTTONS <<-EOF                                                                                                                                                     
1 Say Hello
2 Go Home
3 Make End Effector Trajectory
4 Make Joint Trajectory
5 Execute Joint Trajectory
6 Gripper Move
7 Gripper Homing
8 Gripper Stop
9 Gripper Grasp
12 Joint bar grasp
13 Control bar grasp
14 Box Closing Joint Process Left
15 Box Closing Control Process Left
16 Box Closing Joint Process Right
17 Box Closing Control Process Right
0 Exit                                                                                                                                                                 
EOF

for BUTTON in "${BUTTONS[@]}"
do
    VALUE=("${BUTTON[@]%% *}")
    TEXT=("${BUTTON[@]#* }")
    echo VALUE=${VALUE}
    echo TEXT=${TEXT}

#    convert -size 512x512 -background lightblue -pointsize 100 -fill black -gravity Center caption:"$TEXT" -flatten /tmp/button_${VALUE}.png                              
#    convert -size 256x512 -background lightblue -pointsize 70 -fill black -gravity Center caption:"$TEXT" -flatten /tmp/button_${VALUE}.png                               
    convert -size 80x200 -background lightblue -pointsize 10 -fill black -gravity Center caption:"$TEXT" -flatten /tmp/button_${VALUE}.png

    YADPARAMS+="--button=!/tmp/button_${VALUE}.png:$VALUE "

done

while true
do

    yad --form ${YADPARAMS[@]}
    CODE=$?

    if [[ $CODE == 0 ]]
    then
        break
    fi

    if [[ $CODE == 1 ]]
    then
        break
    fi

    if [[ $CODE == 2 ]]
    then
        rosrun panda_board execute_trajectory_client.py 0 0
	break
    fi

    if [[ $CODE == 3 ]]
    then
        rosrun panda_board execute_trajectory_client.py 0 1
        break
    fi

    if [[ $CODE == 4 ]]
    then
        rosrun panda_board make_ee_trajectory_client.py
	break
    fi

    if [[ $CODE == 5 ]]
    then
        rosrun panda_board make_joint_trajectory_client.py
	break
    fi

    if [[ $CODE == 6 ]]
    then
        rosrun panda_board execute_trajectory_client.py 0 2
	break
    fi

    if [[ $CODE == 13 ]]
    then
        rosrun panda_board make_ee_trajectory_client [0.088, 0, 0.942] [0, 1, 0, 0] [0.2, 0, 0.01] [0, 0.7071063811865475, 0.7071063811865475, 0]
	rosrun panda_board make_joint_trajectory_client 0.01
	rosrun panda_board execute_joint_trajectory_client 0 1
	rosrun panda_board execute_joint_trajectory_client 0 0
	break
    fi

    if [[ $CODE == 14 ]]
    then

        rosrun panda_board make_ee_trajectory_client [0.088, 0, 0.942] [0, 1, 0, 0] [0.549582774, -0.470917235, 0.105077945] [0.6695266785014207, 0.7068179722806072, -0.15689767619864317, 0.165908107214361]
        rosrun panda_board make_joint_trajectory_client 0.01
        rosrun panda_board execute_joint_trajectory_client 0 2
        
	rosrun panda_board make_ee_trajectory_client [0.549582774, -0.470917235, 0.105077945] [0.6695266785014207, 0.7068179722806072, -0.15689767619864317, 0.165908107214361] [0.33861799, -0.47790584, 0.30571764] [0.4263589656895043, 0.30517421083650015, -0.6297034895737142, 0.5732017521215216]
        rosrun panda_board make_joint_trajectory_client 0.01
        rosrun panda_board execute_joint_trajectory_client 0 2

	rosrun panda_board make_ee_trajectory_client [0.33861799, -0.47790584, 0.30571764] [0.4263589656895043, 0.30517421083650015, -0.6297034895737142, 0.5732017521215216] [0.36409323, -0.31025503, 0.31640584] [0.3615162823743919, 0.3823681718967822, -0.6053656257814838, 0.5971875901429163]
        rosrun panda_board make_joint_trajectory_client 0.01
        rosrun panda_board execute_joint_trajectory_client 0 2

        break
    fi

    if [[ $CODE == 15 ]]
    then
        
	rosrun panda_board make_ee_trajectory_client [0.088, 0, 0.942] [0, 1, 0, 0] [0.03675299, -0.50008365, 0.15172953] [0.5453292413855866, 0.6733769579145392, -0.38831883909661696, 0.3136685675075478]
        rosrun panda_board make_joint_trajectory_client 0.01
        rosrun panda_board execute_joint_trajectory_client 0 2

        rosrun panda_board make_ee_trajectory_client [0.03675299, -0.50008365, 0.15172953] [0.5453292413855866, 0.6733769579145392, -0.38831883909661696, 0.3136685675075478] [0.23676345, -0.5027645, 0.32561625] [0.5886922282833736, 0.5460862878861434, 0.45619568830916823, -0.38355797542172515]
        rosrun panda_board make_joint_trajectory_client 0.01
        rosrun panda_board execute_joint_trajectory_client 0 2

        rosrun panda_board make_ee_trajectory_client [0.23676345, -0.5027645, 0.32561625] [0.5886922282833736, 0.5460862878861434, 0.45619568830916823, -0.38355797542172515] [0.21325529, -0.38247205, 0.32692933] [0.5761211384542526, 0.5766075911320828, 0.41722489396226914, -0.4019098251264902]
        rosrun panda_board make_joint_trajectory_client 0.01
        rosrun panda_board execute_joint_trajectory_client 0 2

        break
    fi

    rostopic pub -1 /panda_board/command_ind std_msgs/Int16 $CODE
done
