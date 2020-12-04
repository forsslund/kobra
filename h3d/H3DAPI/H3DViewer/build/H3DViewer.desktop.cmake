[Desktop Entry]

# The type as listed above
Type=Application

# The version of the desktop entry specification to which this file complies
Version=1.0

# The name of the application
Name=H3DViewer

# A comment which can/will be used as a tooltip
Comment=Viewer for H3D API. Supports X3D and vrml files.

# The executable of the application, possibly with arguments.
Exec=H3DViewer

# The name of the icon that will be used to display this entry
Icon=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/H3D/H3DViewer.svg

# Describes whether this application needs to be run in a terminal or not
Terminal=false

# Describes the categories in which this entry should be shown
Categories=Graphics;Viewer;

#List mime types that can be opened by H3DViewer
MimeType=model/vrml;
