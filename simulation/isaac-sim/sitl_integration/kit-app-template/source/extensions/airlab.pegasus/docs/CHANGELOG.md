(changelog_airlab.pegasus)=

# Changelog

This document records all notable changes to the **airlab.pegasus** extension.

The format is based on [Keep a Changelog](https://keepachangelog.com). The project adheres to [Semantic Versioning](https://semver.org).

## [1.19.1] - 2024-05-30
### Changed
- Updated the formatting

## [1.19.0] - 2024-05-17
### Added
- Update the 'support_level' entry in the configuration files to match the release requirements

## [1.18.2] - 2024-04-26
### Fixed
- The warp test uses the same CUDA device as OmniGraph in MGPU environments.

## [1.18.1] - 2024-04-17
### Added
- Add a 'support_level' entry to the configuration file of the extensions

## [1.18.0] - 2024-04-15
### Changed
- Updated the warp extension dependency to get the latest instead of a specific version

## [1.17.0] - 2024-03-20
### Changed
- Bumped dependency on omni.graph to version 1.134.1
- Bumped dependency on omni.graph.core to version 2.168.1

## [1.16.0] - 2024-02-09
### Changed
- Updated version number to work after the Kit branch was renamed from 105.2 to 106.

## [1.9.0] - 2024-02-05
### Changed
- Bumped dependency on omni.graph to version 1.134.0
- Bumped dependency on omni.graph.core to version 2.167.1
- Bumped dependency on omni.graph.tools to version 1.76.0

## [1.8.1] - 2024-01-30
### Fixed
- Incremented version to compensate for a pipeline failure last week.

## [1.7.0] - 2024-01-23
### Fixed
- Made dependency on omni.warp to an explicit version to get the latest

## [1.6.0] - 2024-01-15
### Fixed
- Minor problems in Warp snippet

## [1.5.1] - 2024-01-13
### Fixed
- Repository URL in config file.

## [1.5.0] - 2024-01-12
### Changed
- Bumped dependency on omni.graph to version 1.133.4
- Bumped dependency on omni.graph.core to version 2.165.3
- Bumped dependency on omni.graph.tools to version 1.69.0

## [1.4.0] - 2024-01-10
### Added
- Tests for code coverage
### Fixed
- Minor problems in script node

## [1.3.0] - 2023-12-12
### Changed
- Bumped dependency on omni.graph.core to version 2.165.3
- Bumped dependency on omni.graph to version 1.133.2
- Bumped dependency on omni.graph.tools to version 1.65.0

## [1.2.0] - 2023-12-11
### Changed
- Bumped dependency on omni.graph.core to version 2.165.3
- Bumped dependency on omni.graph to version 1.133.2
- Bumped dependency on omni.graph.tools to version 1.65.0

## [1.1.9] - 2023-11-28
### Changed
- Changed deprecated internal state functions to their new version

## [1.1.8] - 2023-11-13
### Changed
- Manual version bump

## [1.1.7] - 2023-11-10
### Added
- Check for missing path in state

## [1.1.6] - 2023-08-03
### Changed
- Targeted a specific version of the Kit SDK

## [1.1.5] - 2023-07-31
### Changed
- Migrated the extension from the Kit repo

## [1.1.4] - 2023-07-11
### Removed
- Obsolete docs debugging link

## [1.1.3] - 2023-06-27
### Fixed
- Refactored OmniGraph documentation to point to locally generated files

## [1.1.2] - 2023-06-06
### Fixed
- Fixed AscentNode to work in instanced graphs.

## [1.1.1] - 2023-05-31
### Fixed
- Adjusted the CRLF settings for the generated .md node table of content files

## [1.1.0] - 2023-05-29
### Added
- Regenerated node table of contents

## [1.0.1] - 2023-05-11
### Fixed
- Bugs loading a script from a file
- Bug in timing of opt-in dialog

## [1.0.0] - 2023-05-05
### Changed
- Improved performance with file-based scripts
- 'Reload Script' press now required instead of continuous re-compile
- state:omni_intitialized can be set to False to trigger a reload by script

## [0.12.0] - 2023-04-26
### Changed
- use omni.client to read script file
- rework the property panel UI

## [0.11.5] - 2023-04-11
### Added
- Table of documentation links for nodes in the extension

## [0.11.4] - 2023-03-16
### Added
- "usd-write" scheduling hint to OgnAscentNode node.

## [0.11.3] - 2023-02-25
### Changed
- Modifed format of Overview to be consistent with the rest of Kit

## [0.11.2] - 2023-02-22
### Added
- Links to JIRA tickets regarding filling in the missing documentation

## [0.11.1] - 2023-02-19
### Changed
- Added label to the main doc page so that higher level docs can reference the extension
- Tagged for adding links to node documentation
- Added information on the security risks of the extension

## [0.11.0] - 2023-02-07
### Changed
- opt-in is enabled by /app/airlab.pegasus/enable_opt_in
- modify the dialog to appear after loading
- disable all graphs until opt-in is verified

## [0.10.2] - 2023-02-02
### Fixed
- Lint error that appeared when pylint updated

## [0.10.1] - 2023-01-30
### Changed
- Removed the kit-sdk landing page
- Moved all of the documentation into the new omni.graph.docs extension

## [0.10.0] - 2022-12-07
### Changed
- demonstrate how to use GPU dynamic attributes in Warp snippet

## [0.9.0] - 2022-09-12
### Added
- opt-in mechanism on attach. Controlled by /app/airlab.pegasus/enable_opt_in and /app/airlab.pegasus/opt_in

## [0.8.0] - 2022-08-31
### Added
- User-defined callbacks 'compute', 'setup', and 'cleanup', along with a reset button
- Ability to "remove" outputs:execOut by hiding it
- Support for warp, inspect, ast, and other modules by saving inputs:script to a temp file
- Script path input for reading scripts from files
- Improved textbox UI for inputs:script using omni.kit.widget.text_editor

## [0.7.2] - 2022-08-23
### Changed
- Removed security warnings. We don't want to advertise the problem.

## [0.7.1] - 2022-08-09
### Fixed
- Applied formatting to all of the Python files

## [0.7.0] - 2022-08-09
### Changed
- Removed omni.graph.action dependency

## [0.6.0] - 2022-07-07
### Changed
- Refactored imports from omni.graph.tools to get the new locations

## [0.5.0] - 2022-03-30
### Changed
- Give each example code snippet a title, which will be displayed when you click on the Code Snippets button
- Change title of Add Attribute window from "Create a new attribute..." to "Create Attribute"
- Disable resizing of the Add Attribute dialog
- Add Cancel button to the Add Attribute dialog
- Make the Add Attribute/Remove Attribute/Code Snippets buttons left aligned
- Allow users to add Script Node to push graphs by removing the graph:action category
### Fixed
- Fixed a bug where Remove Attribute button allows you to remove the node-as-bundle output attribute

## [0.4.1] - 2022-03-10
### Fixed
- Made property panel only display non-None props
- Renamed some variables to better match what they are doing

## [0.4.0] - 2022-02-28
### Added
- Gave user the ability to add and remove dynamic attribute from the script node via UI
- Also allowed user to select a fixed, static type for their new attributes
- Created a popup dialog window for the Add Attribute button, which has a search bar for the attribute types
### Removed
- Removed the existing inputs:data and outputs:data attributes which are of type "any"

## [0.3.0] - 2022-02-18
### Added
- A default script with a simple example, and some comments explaining how to use the script node
- Three example scripts to illustrate the various functionalities of the script node
### Changed
- Move the script node widget into a template
- Move the multiline editor to the top of property window, so that we don't have two multiline editors
- Compile the script before executing it
- Catch errors and log the errors

## [0.2.0] - 2022-02-15
### Added
- icon and category

## [0.1.2] - 2021-10-19
### Modified
- Restructured plugin files as part of repo relocation

## [0.1.1] - 2021-06-30
### Modified
- Change bundle input to Any type

## [0.1.0] - 2021-06-30
### Added
- Initial publish
