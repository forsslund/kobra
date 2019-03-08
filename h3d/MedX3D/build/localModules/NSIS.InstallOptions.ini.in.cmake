[Settings]
NumFields=${NSIS_OPTIONS_NUMFIELDS}

[Field 1]
Type=label
Text=By default @CPACK_PACKAGE_INSTALL_DIRECTORY@ does not add anything to PATH. Use the options below to add the folder containing binaries to PATH, this could be useful for developers that plan to compile MedX3D and MedX3DDemo.
Left=0
Right=-1
Top=0
Bottom=30

[Field 2]
Type=radiobutton
Text=Do not add the directories to the system PATH
Left=0
Right=-1
Top=30
Bottom=40
State=1

[Field 3]
Type=radiobutton
Text=Add the directories to the system PATH for all users
Left=0
Right=-1
Top=40
Bottom=50
State=0

[Field 4]
Type=radiobutton
Text=Add the directories to the system PATH for current user
Left=0
Right=-1
Top=50
Bottom=60
State=0

[Field 5]
Type=label
Text=By default @CPACK_PACKAGE_INSTALL_DIRECTORY@ tries to register H3D plugin for Internet Explorer, Mozilla Firefox and Opera. Use the options below if any changes to this behaviour is desired. The folder containing MedX3D binaries will be added to path if plugin for Mozilla or Opera is selected.
Left=0
Right=-1
Top=70
Bottom=100

[Field 6]
Type=checkbox
Text=Try to register ActiveX plugin for Internet Explorer.
Left=0
Right=-1
Top=100
Bottom=110
State=@CPACK_REGISTER_PLUGINS@

[Field 7]
Type=checkbox
Text=Try to register plugin for Mozilla Firefox.
Left=0
Right=-1
Top=110
Bottom=120
State=@CPACK_REGISTER_PLUGINS@

[Field 8]
Type=checkbox
Text=Try to register plugin for Opera.
Left=0
Right=-1
Top=120
Bottom=130
State=@CPACK_REGISTER_PLUGINS@
