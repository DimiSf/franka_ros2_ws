1. **Open a terminal and navigate to your workspace**

    ```bash
    cd ~/franka_ros2_ws
    ```

2. **Verify you're on the correct branch (should be `main`)**

    ```bash
    git branch
    ```
    > You should see: `* main`

3. **Pull the latest changes from GitHub**

    ```bash
    git pull origin main
    ```

4. **Stage all modified and new files**

    ```bash
    git add .
    ```

5. **Commit your changes with a descriptive message**

    ```bash
    git commit -m "Describe what you changed"
    ```

6. **Push your committed changes to GitHub**

    ```bash
    git push origin main
    ```
