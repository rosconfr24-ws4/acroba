
#!/usr/bin/env python3

import rospy
import py_trees
from py_trees.blackboard import SetBlackboardVariable
from watch_assembly.behaviors.skills import Locate, MoveTo, OpenGripper, CloseGripper
from watch_assembly.pieces import PieceType, PieceColor



def create_naive_tree(target_color=None):
    root = py_trees.composites.Sequence(name="root")
    root.add_children([
        Locate("locate_b", piece_type=PieceType.BASE.name),         
        Locate("locate_m", piece_type=PieceType.MECHANISM.name), 
        OpenGripper(), 
        MoveTo("move_to_m", "locate_mechanics.found_pose"), 
        CloseGripper(), 
        MoveTo("move_to_b", "locate_base.found_pose"), 
    ])
    return root


if __name__=="__main__":
    rospy.init_node("watch_maker")

    root = create_naive_tree() 
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(15)
    py_trees.logging.level = py_trees.logging.Level.INFO

    # Tick the tree until a terminal state is reached
    done = False
    while not rospy.is_shutdown() and not done:
        tree.tick()
        if tree.root.status == py_trees.common.Status.SUCCESS:
            print("Task succeeded")
            done = True
        elif tree.root.status == py_trees.common.Status.FAILURE:
            print("Task failed")
            done = True
        rospy.sleep(0.5)
    
    
