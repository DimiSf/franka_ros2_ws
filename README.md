# 1. Open a terminal and go to your workspace
cd ~/franka_ros2_ws

# 2. Check you're on the correct branch (main)
git branch
# (should show * main)

# 3. Pull the latest changes from GitHub
git pull origin main

# 4. Stage your changes (this includes any edited/added files)
git add .

# 5. Commit your changes with a clear message
git commit -m "Describe what you changed"

# 6. Push your changes to GitHub
git push origin main
