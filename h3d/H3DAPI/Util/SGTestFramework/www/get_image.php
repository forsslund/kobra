<?php
require('mysql_conf.php');




$image_query = "SELECT %s FROM %s WHERE id=?"; 

if($_GET['type']=="result") {
  echo fetch_image($db, $image_query, "rendering_results", "output_image");
} else if ($_GET['type']=="diff") {
  echo fetch_image($db, $image_query, "rendering_results", "diff_image");
} else if ($_GET['type']=="baseline") {
  echo fetch_image($db, $image_query, "rendering_baselines", "image");
}

function fetch_image($db, $query, $table_name, $column_name) {
  if($image_statement = $db->prepare(sprintf($query, $column_name, $table_name))) {
    $image_statement->bind_param("i", $_GET['id']);
    $image_statement->execute();
    $image_statement->bind_result($image);
    if($image_statement->fetch()) {
    header('Content-Type: image/png');
    header('Content-Disposition: inline; filename="'.$_GET['name'].'"');
      echo $image;
    } else {
      die("ERROR, unable to fetch: " . mysqli_error($db));
    }     
  } else {
   die("ERROR with query: " . mysqli_error($db));
 }
}

?>