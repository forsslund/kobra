# SGTestFramework Installation

## Overview

Both 32bit and 64bit versions of all of this software should work, but you must ensure you use the same architecture for everything except imagemagick (but it's probably safer to use the same architecture for that as well.)

## Windows

### Software to install

 - ImageMagick 6.9.3 Q16 (http://www.imagemagick.org/script/binary-releases.php)

 - Python 2.7 (https://www.python.org/downloads/release/python-2711/) Python 3 could also be used.

 - MySQL Connector C 6.0.2 (https://dev.mysql.com/downloads/connector/c/, make sure you install version 6.0.2 and that it installs into "Program Files (x86)" rather than "Program Files", as the installer for the MySQL-python package might fail otherwise.)

 - Microsoft Visual C++ Compiler for Python 2.7 (https://www.microsoft.com/en-us/download/details.aspx?id=44266)

 - Python for Windows Extensions (http://sourceforge.net/projects/pywin32/)

 - Psutil. Just use pip, you can get it with "python -m pip install psutil"

 - MySQL-python package (provides the MySQLdb module)
   (https://pypi.python.org/pypi/MySQL-python/).
    Can be installed by opening python and running "python -m pip install MySQL-python" from anywhere
    You must ensure you installed the right version of MySQL Connector C and that you installed it into Program Files (x86)
   If python 3 is used then use the pymysql package instead of MySQL-python.


### Required path entries

 - Python27 folder (not bin, installer asks if you want to it add this automatically)
 - ImageMagick folder (not bin, installer asks if you want it to add this automatically)

## Linux

### Ubuntu 19.02

#### Test runner
- Install additional prerequisites for test runner. Assumes you already have Python installed.

```
sudo apt install default-libmysqlclient-dev
sudo python -m pip install MySQL-python
sudo apt install imagemagick
```

#### Results server

NOTE: This section explains in broad terms how to install the results server locally, it does not address security issues relevant for installing a publicly accessible server.

- Install prerequisites for results server: Install a basic LAMP stack and create a virtual host config to serve from the SGTestFramework www directory.

- Follow one of the many online guides to configure a virtual host for the results server.

```
# Install LAMP stack
sudo apt-get install apache2
sudo apt-get install mysql-server
sudo apt-get install php libapache2-mod-php php-mysql

# ... etc
```

- Create the database structure.

```
sudo mysql < '/H3D_install_dir/H3DAPI/Util/SGTestFramework/result_database_structure.sql'
```

- Create database users.

```
sudo mysql

CREATE USER 'ResultReader'@'localhost' IDENTIFIED BY 'results';
GRANT SELECT ON testdatabase.* TO 'ResultReader'@'localhost';

CREATE USER 'ResultWriter'@'localhost' IDENTIFIED BY 'runner';
GRANT ALL PRIVILEGES ON testdatabase.* TO 'ResultWriter'@'localhost';

```

- Specify/customize database details. If you changed the default database name, username or password you must specify them by editing `www/mysql_conf.php`

```
$dbhost = 'localhost';
$dbname = 'testdatabase';
$dbuser = 'ResultReader';
$dbpass = 'results';
```

- If you still see no results, disable the `ONLY_FULL_GROUP_BY` option for MySQL. Edit `/etc/mysql/mysql.conf.d/mysqld.cnf` to set (i.e. remove `ONLY_FULL_GROUP_BY`):
```
sql_mode = "STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_AUTO_CREATE_USER,NO_ENGINE_SUBSTITUTION"
```