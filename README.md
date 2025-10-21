passing condition: 
    In test_system_launch.py file change the params like below:
        goal_x         = os.getenv('GOAL_X', '1.8')
        goal_y         = os.getenv('GOAL_Y', '1.2')




failing condition: 
    In test_system_launch.py file change the params like below:
        goal_x         = os.getenv('GOAL_X', '1.49999')
        goal_y         = os.getenv('GOAL_Y', '2.255136')

