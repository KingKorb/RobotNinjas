# Git and Github
```shell
cd ~/dev_ws/src/RobotNinjas/
git config user.name "<user1 or user2 or otheruser's name>"  # sign up
git config user.email "<user_whoever@email.com>"  # get responsible for the upcoming work
git pull origin main  # download updates from github

# make some changes ...

git add .  # stage changed files, locally
git commit -m "<whatever description regarding the changes>"  # commit changed files, locally
git push origin main  # upload local changes to github
```

# Usage
```shell
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash
ros2 run bringup_ninjasrobot ninjasrobot_core
```
Try to remote control it with `teleop_twist_keyboard`
