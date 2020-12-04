<?php

$dbhost = 'localhost';
$dbname = 'testdatabase';
$dbuser = 'ResultReader';
$dbpass = 'results';
$svnuser = ''; // Needs to have read and write access to $svnpath.
$svnpass = ''; 
$svnpaths = []; // Needs to be a list of full svn urls for checking out the root directory of any of the tests that are shown on this page. Necessary for baseline uploads.

$db = mysqli_connect($dbhost, $dbuser, $dbpass, $dbname);
if(mysqli_connect_errno($db)) {
  echo "Failed to connect to MySQL: " . mysqli_connect_error();
}


?>