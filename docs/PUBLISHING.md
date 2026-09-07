# Publish ASBR to GitHub

Recommended repository: **DanielAgbara/ASBR**. Description: *Robotics algorithms in Python and MATLAB: rigid-body motion, KUKA kinematics, sensor calibration and virtual fixtures.*

## Prepare the first commit

The original code has an MIT license with both authors credited. Bundled robot assets retain their own terms. Original root PDFs, local prototypes, caches and generated output galleries are ignored; selected README figures are in `docs/assets`.

From PowerShell in this project folder:

```powershell
git init -b main
git add .
git status --short
git diff --cached --stat
git commit -m "Prepare ASBR robotics projects for portfolio"
```

Run `git init` only if the directory has not already been initialized. Inspect the staged file list before committing; do not force-add ignored original submissions. Review the attribution of bundled course inputs and robot assets in `THIRD_PARTY_NOTICES.md`.

## Create the remote and upload

On GitHub, sign in as `DanielAgbara` and create an empty repository named **ASBR**. Leave the README, license and gitignore initialization options off because those files already exist locally. Select the intended visibility, then run:

```powershell
git remote add origin https://github.com/DanielAgbara/ASBR.git
git push -u origin main
```

Git may open a browser sign-in through the credential manager. Do not paste passwords or tokens into the project files. If `origin` already exists, inspect `git remote -v` before changing it.

During local preparation, the stored GitHub credential returned HTTP 401. On this Windows installation, refresh it with:

```powershell
git credential-manager github login --username DanielAgbara --force --browser
```

Complete the GitHub sign-in yourself, then run the push command. No repository has been created remotely by the preparation scripts.

Alternatively, with GitHub CLI installed and authenticated, after the local commit:

```powershell
gh repo create DanielAgbara/ASBR --public --source=. --remote=origin --push
```

This follows [GitHub's instructions for locally hosted code](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github) and the [GitHub CLI repository creation reference](https://cli.github.com/manual/gh_repo_create).

## Verify the published repository

Open the root README and each THA README. Check image rendering and video downloads, then clone into a separate directory and try the documented Python commands and MATLAB launcher. Add topics such as `robotics`, `matlab`, `python`, `kinematics`, `sensor-calibration` and `virtual-fixtures` in the repository settings.

## Publish the portfolio update

The website repository is `C:\Users\goz4d\Documents\Websites\DanielAgbara.github.io`. The ASBR addition should include `projects/asbr.html`, its media, and a homepage card. Publish the ASBR source repository first so the case study's source links resolve. Review the website diff, commit those specific files and push its existing branch. Keep unrelated changes out of the commit.
