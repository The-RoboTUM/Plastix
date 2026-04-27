# PlastiX - Main Repository
Welcome to the PlastiX project which is part of CirQmind. Here you will find the [Wiki](https://gitex.itq.de/cirqmind/PlastiX/-/wikis/home), the [Issue Board](https://gitex.itq.de/cirqmind/PlastiX/-/boards) and the project files in their respective branches.
Hallo
## Git Usage
### Cloning the Repository
1. Open a terminal window (or do this e.g. via the VS Code Extension).
2. Navigate to where you want to clone the repository, e.g. `/Documents/ITQ/`
```bash
cd .../Documents/ITQ
```
3. Clone the repository. You might get prompted for a login. If so, quickly go to the `Git Authentication` section below.
```bash
git clone https://gitex.itq.de/cirqmind/PlastiX.git
```
4. Authenticate if prompted, otherwise you should now have a new folder named `PlastiX`.
5. Enter the folder and switch to your branch (might not exist yet, contact your Team Leader):
```bash
cd PlastiX
git checkout <your-branch>
```

### Synchronizing Files
- Always edit files in your branch on your **local** repository.
- Always run `git pull` before making changes or uploading them to ensure you have the latest files.
- Run `git add .` to stage all changes in the current folder and all subfolders
- Run `git commit -m "Description of changes"` to create a commit
- Run `git push` to upload all commits to the remote repository.


### Git Authentication
In order to be able to push/pull to/from the remote repository you will have to login to GitLab locally. The easiest way is by creating an Access Token in the web interface and using it to login
1. In the [GitLab Web Interface](https://gitex.itq.de/cirqmind/PlastiX/-/tree/main) click your profile picture and navigate to `Preferences -> Personal access tokens`
2. Click `Add new token`.
3. Give it a name, e.g. `Full Access - Home Laptop` and set the expiration date to at least until the end of February 2026.
4. Check all the boxes (unless you have security concerns).
5. Click `Create token`.
6. The Token will appear hidden. Copy it now and use it for authentication. **After this, you will not be able to copy it again!**. The Token is not intended for repeated use, so don't save it, just use it once.
7. To use it as a login somewhere, set the **username** to `oauth2` and the **password** to your token.

# Project Information

## High-Level System Requirements
1. PlastiX robots and drones must autonomously detect and collect plastic waste together
2. The system must be highly innovative, look astonishing to impress industry partners and fair visitors
3. The system should be voice controlled and accessible via an internet connection
4. Everything must work outdoors and also indoors (at fairs)
5. All robots must be able to simulate inputs and outputs
6. All robots should have digital twins in one shared enviroment

## Links
 - [Nextcloud](https://nextcloud.itq.de/f/1177)