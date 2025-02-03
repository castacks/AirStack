# Kit Base Editor App Template

![Kit Base Editor Image](../../../readme-assets/kit_base_editor.png)

**Based On:** `Omniverse Kit SDK 106.0`

## Overview

The Kit Base Editor App Template provides a minimal starting point for developers aiming to create interactive 3D applications within the NVIDIA Omniverse ecosystem. This template simplifies the process of crafting applications capable of loading, manipulating, and rendering Open Universal Scene Description (OpenUSD) content via a graphical user interface.

### Use Cases
This template is ideal for developers looking to build:

- 3D viewing and editing applications and tools from a functional, minimal starting point.


### Key Features

- Scene loading
- RTX Renderer
- Basic UI for manipulating and exploring 3D scenes.

## Usage

This section provides instructions for the setup and use of the Kit Base Editor Application Template.

### Getting Started

To get started with the Kit Base Editor, ensure your development environment meets the prerequisites outlined in the top-level README.

#### Cloning the Repository

```bash
git clone https://github.com/NVIDIA-Omniverse/kit-app-template.git
cd kit-app-template
```

#### Create New Application

**Linux:**
```bash
./repo.sh template new
```

**Windows:**
```powershell
.\repo.bat template new
```

Follow the prompt instructions:
- **? Select with arrow keys what you want to create:** Application
- **? Select with arrow keys your desired template:** Kit Base Editor
- **? Enter name of application .kit file [name-spaced, lowercase, alphanumeric]:** [set application name]
- **? Enter application_display_name:** [set application display name]
- **? Enter version:**: [set application version]

### Build and Launch

#### Build your application using the provided build scripts:
Note that the build step will build all applications contained in the `source` directory. Outside of initial experimentation, it is recommended that you build only the application you are actively developing.

**Linux:**
```bash
./repo.sh build
```
**Windows:**
```powershell
.\repo.bat build
```

 If you experience issues related to build, please see the [Usage and Troubleshooting](readme-assets/additional-docs/usage_and_troubleshooting.md) section for additional information.

#### Launch your application:

**Linux:**
```bash
./repo.sh launch
```
**Windows:**
```powershell
.\repo.bat launch
```

**? Select with arrow keys which App would you like to launch:** [Select the desired editor application]

***NOTE:* The initial startup may take 5 to 8 minutes as shaders compile for the first time. After initial shader compilation, startup time will reduce dramatically**

### Testing
Applications and their associated extensions can be tested using the `repo test` tooling provided. Each application template includes an initial test suite that can be run to verify the application's functionality.

**Note:** Testing will only be run on applications and extensions within the build directory. **A successful build is required before testing.**

**Linux:**
```bash
./repo.sh test
```

**Windows:**
```powershell
.\repo.bat test
```

### Customization

#### Enable Extension
- On launch of the Application enable the developer bundle by adding the `--dev-bundle` or `-d` flag to the launch command.

    **Linux:**
    ```bash
    ./repo.sh launch --dev-bundle
    ```
    **Windows:**
    ```powershell
    .\repo.bat launch --dev-bundle
    ```
- From the running application select `Developer` > `Utilities` > `Extensions`

- Browse and enable extensions of interest from the Extension Manager.
    - Enabling the extensions within the Extension Manager UI will allow you to try out the features of the extension in the currently running application.

    - To permanently add the extension to the application, you will need to add the extension to the `.kit` file. For example, adding the Layer View extension would require adding `omni.kit.widget.layers` to the dependencies section of the `.kit` file.

- For additional information on the Developer Bundle Extensions, refer to the [BETA - Developer Bundle Extensions](readme-assets/additional-docs/developer_bundle_extensions.md) documentation.

#### Create Custom Extension

**Linux:**
```bash
./repo.sh template new
```

**Windows:**
```powershell
.\repo.bat template new
```

Follow the prompt instructions:
- **? Select with arrow keys what you want to create:** Extension
- **? Select with arrow keys your desired template:**: [choose extension template]
- **? Enter name of extension [name-spaced, lowercase, alphanumeric]:**: [set extension name]
- **? Enter extension_display_name:**: [set extension display name]
- **? Enter version:**: [set extension version]


#### Adding Extension to .kit File
**Importantly** For an extension to become a persistent part of an application, the extension will need to be added to the `.kit` file.

```toml
[dependencies]
"extension.name" = {}
```

#### Build with New Extensions
After a new extension has been added to the `.kit` file, the application should be rebuilt to ensure extensions are populated to the build directory.


### Packaging and Deployment

For deploying your application, create a deployable package using the `package` command:

**Linux:**
```bash
./repo.sh package --name [package_name]
```
**Windows:**
```powershell
.\repo.bat package --name [package_name]
```

This will bundle your application into a distributable format, ready for deployment on compatible platforms.

:warning: **Important Note for Packaging:** Because the packaging operation will package everything within the `source/` directory the package version will need to be set independently of a given `kit` file.  **The version is set within the `tools/VERSION.md` file.**

## Additional Learning

- [Kit App Template Companion Tutorial](https://docs.omniverse.nvidia.com/kit/docs/kit-app-template/latest/docs/intro.html)
