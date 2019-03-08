<?php

error_reporting(E_ALL);

require('mysql_conf.php');

ini_set('memory_limit', '512M');

$test_run_id = $_GET['test_run_id'];
  
$perf_query = sprintf("
(SELECT performance_results.id AS id,test_runs.timestamp,server_id,build_name,hardware_name,test_run_id,file_id,filename,case_id,
        'performance' AS result_type,case_name,performance_results.step_id,step_name, full_profiling_data, test_cases.svn_url_x3d, test_cases.svn_url_script, test_files.description
 FROM   test_runs
        left join performance_results
               ON performance_results.test_run_id = test_runs.id
        join test_cases
          ON performance_results.case_id = test_cases.id
        join test_steps
          ON performance_results.step_id = test_steps.id
        join test_files
          ON performance_results.file_id = test_files.id
        join servers
          ON test_runs.server_id = servers.id
 WHERE  test_runs.id = %d
)", $test_run_id);

$render_query = sprintf("
(SELECT rendering_results.id AS id,test_runs.timestamp,server_id,build_name,hardware_name,rendering_results.test_run_id,
        rendering_results.file_id,filename,rendering_results.case_id,'rendering' AS result_type,case_name,
        rendering_results.step_id,step_name,success,new_failure, error_type, diff_pixels, threshold, test_cases.svn_url_x3d, test_cases.svn_url_script, test_files.description,
        (SELECT id FROM rendering_baselines
          WHERE rendering_results.file_id = rendering_baselines.file_id
            AND rendering_results.case_id = rendering_baselines.case_id
            AND rendering_results.step_id = rendering_baselines.step_id
            AND test_runs.timestamp >= rendering_baselines.timestamp
          ORDER BY rendering_baselines.timestamp DESC LIMIT 1) as baseline_id
 FROM   test_runs
        left join rendering_results
               ON rendering_results.test_run_id = test_runs.id
        join test_cases
          ON rendering_results.case_id = test_cases.id
        join test_steps
          ON rendering_results.step_id = test_steps.id
        join test_files
          ON rendering_results.file_id = test_files.id
        join servers
          ON test_runs.server_id = servers.id
 WHERE  test_runs.id = %d
 GROUP  BY case_id,file_id,step_id
)", $test_run_id);

$console_query = sprintf("
(SELECT console_results.id AS id,test_runs.timestamp,server_id,build_name,hardware_name,console_results.test_run_id,
        console_results.file_id,
        filename,console_results.case_id,'console' AS result_type,case_name,console_results.step_id,step_name,success,new_failure, error_type,
        output AS text_output,baseline AS text_baseline, diff AS text_diff, test_cases.svn_url_x3d, test_cases.svn_url_script, test_files.description
 FROM   test_runs
        left join console_results
               ON console_results.test_run_id = test_runs.id
        join test_cases
          ON console_results.case_id = test_cases.id
        join test_steps
          ON console_results.step_id = test_steps.id
        join test_files
          ON console_results.file_id = test_files.id
        join servers
          ON test_runs.server_id = servers.id
 WHERE  test_runs.id = %d
 GROUP  BY case_id,file_id,step_id
 )", $test_run_id);
 
$custom_query = sprintf("
(SELECT custom_results.id AS id,test_runs.timestamp,server_id,build_name,hardware_name,custom_results.test_run_id,
        custom_results.file_id,
        filename,custom_results.case_id,'custom' AS result_type,case_name,custom_results.step_id,step_name,success,new_failure, error_type,
        output AS text_output,baseline AS text_baseline, diff AS text_diff, test_cases.svn_url_x3d, test_cases.svn_url_script, test_files.description
 FROM   test_runs
        left join custom_results
               ON custom_results.test_run_id = test_runs.id
        join test_cases
          ON custom_results.case_id = test_cases.id
        join test_steps
          ON custom_results.step_id = test_steps.id
        join test_files
          ON custom_results.file_id = test_files.id
        join servers
          ON test_runs.server_id = servers.id
 WHERE  test_runs.id = %d
 GROUP  BY case_id,file_id,step_id
 )", $test_run_id);
 
 
$error_query = sprintf("
(SELECT error_results.id AS id,test_runs.timestamp,server_id,build_name,hardware_name,error_results.test_run_id,
        error_results.file_id,new_failure, error_type,
        filename,error_results.case_id,'error' AS result_type,case_name,error_results.step_id,step_name, stdout, stderr, test_cases.svn_url_x3d, test_cases.svn_url_script, test_files.description
 FROM   test_runs
        left join error_results
               ON error_results.test_run_id = test_runs.id
        join test_cases
          ON error_results.case_id = test_cases.id
        join test_steps
          ON error_results.step_id = test_steps.id
        join test_files
          ON error_results.file_id = test_files.id
        join servers
          ON test_runs.server_id = servers.id
 WHERE test_runs.id = %d
 GROUP BY case_id,file_id,step_id)", $test_run_id);


function getCategoryDescription($db, $path) {
  $category_query = "SELECT description FROM test_categories WHERE path = '" . $path . "';";
  if(!$category_fetch = mysqli_query($db, $category_query)) {
      die("ERROR: " . mysqli_error($db));
  }
  if(mysqli_num_rows($category_fetch) > 0) {
    $cat_row = mysqli_fetch_assoc($category_fetch);
    return $cat_row['description'];
  } else {
    return "";
  }
}

$error_rows = fetch_result($db, $error_query);
$console_rows = fetch_result($db, $console_query);
$custom_rows = fetch_result($db, $custom_query);
$render_rows = fetch_result($db, $render_query);
if(isset($_GET['get_perf']) && $_GET['get_perf']) {
  $perf_rows = fetch_result($db, $perf_query);
}
//echo json_encode($error_rows);





$data = array();
$data = generate_results($db, $data, $error_rows);
$data = generate_results($db, $data, $console_rows);
$data = generate_results($db, $data, $custom_rows);
$data = generate_results($db, $data, $render_rows);
if(isset($_GET['get_perf']) && $_GET['get_perf']) {
  $data = generate_results($db, $data, $perf_rows);
}
if(count($data) == 0) {
$testcase = array(
  "name"   => "No results found",
  "testcases" => array(
    "step_name"  => "",
    "filename"   => "Error",
    "result_type" => "ignore")
  );  
  $data = array($testcase);
}

// After this is done we've successfully built our object and just need to convert it to json.
echo json_encode($data);
	
	
function fetch_result($db, $query) {
  if(!$fetch_result = mysqli_query($db, $query)) {
		die("ERROR: " . mysqli_error($db));
	}	
  $fetched_data = array();

  while($row = mysqli_fetch_assoc($fetch_result)) {
    array_push($fetched_data, $row);
  }
  return $fetched_data;
}	
	
function generate_results($db, $data, $fetched_data) {
 
/*
    Tree-building algorithm:

    for every row:
      loop through the $data array and look for an element with a name property that matches the first name in the filename path.
      if it doesn't exist
        create and add it to data (create with a children array if it isn't the last part of the filename path, otherwise with a testcases array)
        point $node at the newly added node
      else
        point $node at it
        
      if there is more than one element in the filename path
        remove the first element of the filename path
      
      while there's more than 1 parts left of the filename path
        if the current node has a child with a name that matches the first part of the filename path
          point $node at that child node
        else
          create a new node, same as before, and add it to that children array
          point $node at the new node
        remove the first element of the filename path
      
      if node contains a children array
        if node children contains an element for this testcase
          point $node at that element
        else
          create an element for this testcase
          point $node at that element
      else
        //if we get here then that means this is a top-level leaf and we can just proceed to the end of the code
      
      add the data to $node['testcases']
        
*/
  for($fetch_index = 0; $fetch_index < count($fetched_data); $fetch_index++) {
     $row = $fetched_data[$fetch_index];
//    echo "<br/>".json_encode($row, JSON_PARTIAL_OUTPUT_ON_ERROR)."</br>";
    $filename = $row['filename'];
    $category_structure = explode('/', $filename);
    // Store a reference to the root of the data tree. We'll move this reference down the tree until we reach the part where the leaf should go,
    // creating any missing nodes on the way there.
    // NOTE: json_encode won't output an array if there are any named elements in it. It'll turn those into objects.
    $node = &$data;
    $path = $category_structure[0];
    $index = -1;
    for($i = 0; $i < count($data); $i++) {
      if(strcmp($data[$i]['name'], $category_structure[0]) == 0) {
        $index = $i;
        break;
      }
    }
    if($index == -1) { // We don't have a top-level entry for this category or testcase, so let's add it.
      if(count($category_structure) > 1) {
        $category_description = getCategoryDescription($db, $path);
        $node = &$data[array_push($data, array("name" => $category_structure[0], "description" => $category_description, "children" => array()))-1];
      } else {
        $category_description = getCategoryDescription($db, $path);
        $node = &$data[array_push($data, array("name" => $category_structure[0], "description" => $category_description, "testcases" => array()))-1];
      }
    } else {
      $node = &$data[$index];
    }
    if(count($category_structure) > 1)
      array_shift($category_structure);
      $path = $path ."/".$category_structure[0];
//    echo '</br>it is now ' . json_encode($category_structure) ."</br>";
          
//     Go through every folder specified in the casename path and build our array structure.
    while(count($category_structure) > 1) {
      $index = -1;
      for($i = 0; $i < count($node['children']); $i++) {
        if((strcmp($node['children'][$i]['name'], $category_structure[0]) == 0)) {
          $index = $i;
          break;
        }      
      }
      if($index > -1) {
        $node = &$node['children'][$i];
      } else {     
        $category_description = getCategoryDescription($db, $path);
        $new_node = array("name" => $category_structure[0], "description" => $category_description, "children" => array(), 'success' => true);
        $node = &$node['children'][array_push($node['children'], $new_node)-1];
//    echo "node is: </br>" . json_encode($node) . "</br>";
      }
      // Remove the first element from $category_structure
      array_shift($category_structure); 
    }
    //if node contains a children array then it is the category in which the testcase leaf should be
    if(array_key_exists('children', $node)) {
      
      $index = -1;
      for($i = 0; $i < count($node['children']); $i++) {
        if((strcmp($node['children'][$i]['name'], $category_structure[0]) == 0)) {
          $index = $i;
          break;
        }      
      }
      //if node children contains an element for this testfile
      if($index > -1) {
        //point $node at that element
        $node = &$node['children'][$index];
      } else {
        //create and push an element for this testfile into the array
        //point $node at that element
        $category_description = getCategoryDescription($db, $path);
        $new_node = array( 'name' => $category_structure[0], "description" => $category_description, 'testcases' => array(), 'success' => true);
        $node = &$node['children'][array_push($node['children'], $new_node)-1];
        }
    }
    $node['description'] = $row['description'];
    $testcase = array(
      "id" => $row['id'],
      "name"   => $row['case_name'],
      "filename"   => $row['filename'],
      "result_type"   => $row['result_type'],
      "test_run_id" => $row['test_run_id'],
      "file_id" => $row['file_id'],
      "case_id" => $row['case_id'],
      "step_id" => $row['step_id'],
      "server_id"=> $row['server_id'],
      "build_name" => $row['build_name'],
      "hardware_name"=> $row['hardware_name'],
      "time"   => $row['timestamp'],
      "svn_url_x3d" => $row['svn_url_x3d'],
      "svn_url_script" => $row['svn_url_script'],
      "success" => 'Y',
      "new_failure" => 'N',
      "error_type" => 'NO_ERROR',
      "diff_pixels" => 0,
      "threshold" => 0
      );
    $testcase["step_name"] = $row['step_name'];
    
    if($row['result_type'] == "rendering") {
      if($row['success'] =='N') {
        $node['success'] = false;
      }
      $testcase["success"] = $row['success'];
      if($row['new_failure'] == null) { // If new_failure is null then the test was run with an old version of the SGTestFramework.
        $testcase["new_failure"] = 'Y'; // Our fallback is to always tag errors as new in that case
      } else {
        $testcase["new_failure"] = $row['new_failure'];
      }
      $testcase["error_type"] = $row['error_type'];
      $testcase["baseline_id"] = $row['baseline_id'];
      $testcase["diff_pixels"] = $row['diff_pixels'];
      $testcase["threshold"] = $row['threshold'];
    }
    else if($row['result_type'] == "performance") {
      $query = "SELECT * FROM performance_result_data WHERE performance_result_id=".$row['id']." ORDER BY id ASC";
      if(!$perf_fetch = mysqli_query($db, $query)) {
      		die("ERROR: " . mysqli_error($db));
      }
      
      $testcase['full'] = $row['full_profiling_data'];
//      echo "case name: ".$testcase['name']. "</br>";
//      echo json_encode($row, JSON_PARTIAL_OUTPUT_ON_ERROR)."<br/><br/>";
	  $recur = function(&$perf_fetch, &$parent, &$perf_row) use (&$recur, $db, $row, $testcase) {
//	    echo "parent is ".$parent['id']."<br/>";
      if($perf_row != null) {            
        $hist_query = "SELECT timestamp, mean, percent, build_name,hardware_name FROM performance_result_data JOIN performance_results JOIN test_runs ON performance_results.id=performance_result_data.performance_result_id AND performance_results.test_run_id = test_runs.id JOIN servers ON test_runs.server_id=servers.id WHERE case_id=".$row['case_id']." AND step_id=".$row['step_id']." AND identifier=\"".$perf_row['identifier']."\" AND timestamp <= '".$row['timestamp']."' ORDER BY timestamp, performance_result_data.id ASC";
//        print "query: ".$hist_query."<br/></br>";
        if(!$hist_fetch = mysqli_query($db, $hist_query)) {
            die("ERROR: " . mysqli_error($db));
        }
        $history = null;
        if(mysqli_num_rows($hist_fetch) > 0) {
          $history = array();
          while($hist_row = mysqli_fetch_assoc($hist_fetch)) {
            array_push($history, array(
              "timestamp" => $hist_row['timestamp'],
              "mean" => $hist_row['mean'],
              "percent" => $hist_row['percent'],
              "build_name" => $hist_row['build_name'],
              "hardware_name" => $hist_row['hardware_name']
            ));
          }
        }
//        echo json_encode($perf_row['identifier'])."->";
        if($perf_row['level'] > $parent['level']) {
//          echo "down->";
          array_push($parent['children'], array(
            'level' => $perf_row['level'],
            'id' => $perf_row['identifier'],
//            'mean' => $perf_row['mean'],
//            'percent' => $perf_row['percent'],
            'parent' => &$parent,
            'children' => array(),
            'data' => $history
            ));
//            echo "<br/>added ".$perf_row['identifier']." to ".$parent['id']."<br/>";
            $dummy = null;
          $recur($perf_fetch, $parent['children'][count($parent['children'])-1], $dummy);
        } else if ($perf_row['level'] <= $parent['level']){
//          echo "up->";
          $recur($perf_fetch, $parent['parent'], $perf_row);
        } 
      } else {
        while($perf_row = mysqli_fetch_assoc($perf_fetch)) {
//          echo "<br/>".json_encode($perf_row, JSON_PARTIAL_OUTPUT_ON_ERROR)."<br/><br/>"; 
//          echo "fetched<br/>";
          $recur($perf_fetch, $parent, $perf_row);
        }
      }
	  };

      if($perf_row = mysqli_fetch_assoc($perf_fetch)) {        
//         echo json_encode($perf_row, JSON_PARTIAL_OUTPUT_ON_ERROR)."<br/><br/>";                                       // JSON_PARTIAL_OUTPUT_ON_ERROR
      
        $hist_query = "SELECT timestamp, mean, percent, build_name,hardware_name FROM performance_result_data JOIN performance_results JOIN test_runs ON performance_results.id=performance_result_data.performance_result_id AND performance_results.test_run_id = test_runs.id JOIN servers ON test_runs.server_id=servers.id WHERE case_id=".$row['case_id']." AND step_id=".$row['step_id']." AND identifier=\"".$perf_row['identifier']."\" AND timestamp <= '".$row['timestamp']."' ORDER BY timestamp, performance_result_data.id ASC";
//        echo $hist_query."<br/></br>";
//        echo  "query: ".$hist_query."<br/></br>";
        if(!$hist_fetch = mysqli_query($db, $hist_query)) {
            die("ERROR: " . mysqli_error($db));
        }
        $history = null;
        if(mysqli_num_rows($hist_fetch) > 0) {
          $history = array();
          while($hist_row = mysqli_fetch_assoc($hist_fetch)) {
            array_push($history, array(
              "timestamp" => $hist_row['timestamp'],
              "mean" => $hist_row['mean'],
              "percent" => $hist_row['percent'],
              "build_name" => $hist_row['build_name'],
              "hardware_name" => $hist_row['hardware_name']
            ));
          }
        }
              
        $testcase['profiler_data'] = array(
          'level' => $perf_row['level'],
          'id' => $perf_row['identifier'],
          'mean' => $perf_row['mean'],
          'percent' => $perf_row['percent'],
          'parent' => null,
          'children' => array(),
          'data' => $history
          );
          
          
        $dummy = null;
      //  $recur($perf_fetch, $testcase['profiler_data'], $dummy);

      $parent_nuller = function(&$node) use (&$parent_nuller) {
        if($node == null) {
          return;
        }
        unset($node['parent']);
        if(count($node['children']) == 0) {
          unset($node['children']);
        } else {
          foreach($node['children'] as &$child) {
            $parent_nuller($child);
          }
        }
      };
//      echo json_encode($testcase['profiler_data'], JSON_PARTIAL_OUTPUT_ON_ERROR)."<br/><br/></br>";
      $parent_nuller($testcase['profiler_data']);

      }
      
     
    }
    else if ($row['result_type'] == 'console') {
      if($row['success'] =='N') {
        $node['success'] = false;
      }
      $testcase["success"] = $row['success'];

      if($row['new_failure'] == null) { // If new_failure is null then the test was run with an old version of the SGTestFramework.
        $testcase["new_failure"] = 'Y'; // Our fallback is to always tag errors as new in that case
      } else {
        $testcase["new_failure"] = $row['new_failure'];
      }
      
      $testcase["error_type"] = $row['error_type'];
      $testcase["text_output"] = $row["text_output"];
      $testcase["text_baseline"] = $row["text_baseline"];
      $testcase["text_diff"] = $row["text_diff"];
    } else if ($row['result_type'] == 'custom') {
      if($row['success'] =='N') {
        $node['success'] = false;
      }
      $testcase["success"] = $row['success'];

      if($row['new_failure'] == null) { // If new_failure is null then the test was run with an old version of the SGTestFramework.
        $testcase["new_failure"] = 'Y'; // Our fallback is to always tag errors as new in that case
      } else {
        $testcase["new_failure"] = $row['new_failure'];
      }

      $testcase["error_type"] = $row['error_type'];
      $testcase["text_output"] = $row["text_output"];
      $testcase["text_baseline"] = $row["text_baseline"];
      $testcase["text_diff"] = $row["text_diff"];
    } else if ($row['result_type'] == 'error') {
      $node['success'] = false;
      $testcase["success"] = 'N';
      $testcase["stdout"] = $row['stdout'];
      $testcase["stderr"] = $row['stderr'];

      if($row['new_failure'] == null) { // If new_failure is null then the test was run with an old version of the SGTestFramework.
        $testcase["new_failure"] = 'Y'; // Our fallback is to always tag errors as new in that case
      } else {
        $testcase["new_failure"] = $row['new_failure'];
      }

      $testcase["error_type"] = $row['error_type'];
    } 
//     All that remains now is to push the testcase to the node's testcases array
    $target = 0;
    #print $testcase['name']."<br/>";
    while($target < count($node['testcases'])) {
      $namediff = strcasecmp($node['testcases'][$target]['name'], $testcase['name']);
      if($namediff >= 0) {
        if(($namediff > 0) && ($node['testcases'][$target]['id'] <= $testcase['id'])) {
          break;
        } else if ($namediff == 0){
          $target++;
          break;
        }
      }
      $target++;
    }
    array_splice($node['testcases'], $target, 0, array($testcase));

  }
  return $data;
}



?>