[Settings]
NumFields=6

[Field 1]
Type=label
Text=By default @CPACK_PACKAGE_INSTALL_DIRECTORY@ adds two directories to the system PATH. @CPACK_PACKAGE_INSTALL_DIRECTORY@ might not run if the directories are not added to PATH.
Left=0
Right=-1
Top=0
Bottom=20

[Field 2]
Type=radiobutton
Text=Do not add @CPACK_PACKAGE_INSTALL_DIRECTORY@ to the system PATH
Left=0
Right=-1
Top=30
Bottom=40
State=0

[Field 3]
Type=radiobutton
Text=Add @CPACK_PACKAGE_INSTALL_DIRECTORY@ to the system PATH for all users
Left=0
Right=-1
Top=40
Bottom=50
State=0

[Field 4]
Type=radiobutton
Text=Add @CPACK_PACKAGE_INSTALL_DIRECTORY@ to the system PATH for current user
Left=0
Right=-1
Top=50
Bottom=60
State=1

[Field 5]
Type=checkbox 
Text=Install CMake
Left=0
Right=-1
Top=60
Bottom=70
State=0

[Field 6]
Type=checkbox
Text=Use 64 bit binaries.
Left=0
Right=-1
Top=70
Bottom=80
State=1
