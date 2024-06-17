VAR=${1:-bin_picking}

source $VENV_FOLDER/.pyenv/versions/$VAR/bin/activate
screen -S pr_get_rgbd_vg -d -m rosrun skills_vg pr_get_rgbd.py &
screen -S pr_depth_to_pointcloud_vg -d -m rosrun skills pr_depth_to_pointcloud.py &
screen -S test_get_rgbd_vg -d -m rosrun skills_vg test_get_rgbd.py &
screen -S test_depth_to_pointcloud_vg -d -m rosrun skills test_depth_to_pointcloud.py

