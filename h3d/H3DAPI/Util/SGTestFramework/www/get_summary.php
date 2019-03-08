<?php

error_reporting(E_ALL);

require('mysql_conf.php');

ini_set('memory_limit', '512M');

$test_run_id = $_GET['test_run_id'];

$summary = generate_summary($db, $test_run_id);

// After this is done we've successfully built our object and just need to convert it to json.
echo json_encode($summary);	
	

// Just a function that wraps around fetching the data from the database with a given query
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

		

// Given the output from one of the error counting queries, this function determines if there is already an entry for that test category in $summary, otherwise it adds it.
// If it exists then it adds the currently stored counters to the counters in $data and stores the new value.
function add_to_summary($summary, $data) {
  
  for($i = 0; $i < count($data); ++$i) {
    $success_count = 0;
    $step_success_count = 0;
    $fail_count = 0;
    $step_fail_count = 0;
    $error_count = 0;
    $category_name = $data[$i]['filename'];
    if (array_key_exists($category_name, $summary)) {
      $success_count = $summary[$category_name]['success_count'];
      $fail_count = $summary[$category_name]['fail_count'];
      $error_count = $summary[$category_name]['error_count'];
      $step_success_count = $summary[$category_name]['step_success_count'];
      $step_fail_count = $summary[$category_name]['step_fail_count'];
    } else {
      $summary[$category_name] = array(
      'success_count' => 0,
      'fail_count' => 0,
      'step_success_count' => 0,
      'step_fail_count' => 0,
      'error_count' => 0,
      'included_cases' => array(),
      'successful' => true);
    }
    $succeeded = false;
    if(array_key_exists("Y", $data[$i])) {
      if($data[$i]['Y'] > 0)
        $succeeded = true;
      $step_success_count +=  $data[$i]['Y'];
    }
      
    if(array_key_exists("N", $data[$i])) {
      if($data[$i]['N'] > 0)
        $succeeded = false;
      $step_fail_count +=  $data[$i]['N'];
    }
    
    if(array_key_exists('Error', $data[$i])) {
      $succeeded = false;
      $error_count += $data[$i]['Error'];
    }
    
    
    // Check if we've already read some result for this case before
    if(!array_key_exists($data[$i]['case_id'], $summary[$category_name]['included_cases'])) {
      // if we haven't then we just add to our success or fail counter and store the success state of this result we're looking at
      if($succeeded) {
        $success_count++;
      } else {
        $fail_count++;
      }
      $summary[$category_name]['included_cases'][$data[$i]['case_id']] = $succeeded;
    } else {
      // If we have counted this case before then the only case we need to take care of is that of it previously being successful and now no longer being successful (since a failed test case can't become unfailed, but we might find results that show it failed after initially only seeing results that succeeded.
      // So in case it failed nowe we check if it succeeded before, in which case we subtract from successes and increment failures and just store false, to indicate that we found at least one result for this test that didn't succeed.
      if(!$succeeded && $summary[$category_name]['included_cases'][$data[$i]['case_id']]) {
        $success_count--;
        $fail_count++;
        $summary[$category_name]['included_cases'][$data[$i]['case_id']] = false;
      }
    }
    
    
    $summary[$category_name]['success_count'] = $success_count;
    $summary[$category_name]['fail_count'] = $fail_count;
    $summary[$category_name]['step_success_count'] = $step_success_count;
    $summary[$category_name]['step_fail_count'] = $step_fail_count;
    $summary[$category_name]['error_count'] = $error_count;
    $summary[$category_name]['successful'] = ($fail_count + $error_count) == 0;
  }
  
  return $summary;
}

// Grabs the number of successful and failed test steps, as well as the number of error results, for every test result that is part of the specified test run.
// It counts the number of Y and N rows, and groups them all by filename to get us the summary.
// It uses add_to_summary to actually store the data.
function generate_summary($db, $test_run_id) {
  $cons_query = "SELECT case_id, filename, count(case when console_results.success = 'Y' then case_id end) as Y, count(case when console_results.success = 'N' then case_id end) as N FROM console_results JOIN test_cases ON case_id=test_cases.id JOIN test_files ON console_results.file_id = test_files.id WHERE test_run_id=" . $test_run_id . " GROUP BY filename, case_id;";
  $cust_query = "SELECT case_id, filename, count(case when custom_results.success = 'Y' then case_id end) as Y, count(case when custom_results.success = 'N' then case_id end) as N FROM custom_results JOIN test_cases ON case_id=test_cases.id JOIN test_files ON custom_results.file_id = test_files.id WHERE test_run_id=" . $test_run_id . " GROUP BY filename, case_id;";
  $rend_query = "SELECT case_id, filename, count(case when rendering_results.success = 'Y' then case_id end) as Y, count(case when rendering_results.success = 'N' then case_id end) as N FROM rendering_results JOIN test_cases ON case_id=test_cases.id JOIN test_files ON rendering_results.file_id = test_files.id WHERE test_run_id=" . $test_run_id . " GROUP BY filename, case_id;";
  $err_query = "SELECT case_id, filename, count(*) as Error FROM error_results JOIN test_cases ON case_id=test_cases.id JOIN test_files ON error_results.file_id = test_files.id WHERE test_run_id=" . $test_run_id . " GROUP BY filename, case_id;";


  $summary = array();  
  $summary = add_to_summary($summary, fetch_result($db, $cons_query));
  $summary = add_to_summary($summary, fetch_result($db, $cust_query));
  $summary = add_to_summary($summary, fetch_result($db, $rend_query));
  $summary = add_to_summary($summary, fetch_result($db, $err_query));  
  return $summary;
}
