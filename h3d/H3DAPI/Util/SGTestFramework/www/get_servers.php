<?php


require('mysql_conf.php');

$query = "SELECT * FROM servers ORDER BY build_name, hardware_name ASC";

$data = generate_results($db, $query);

echo json_encode($data, 128);
	
function generate_results($db, $query){	
  if(!$fetch_result = mysqli_query($db, $query)) {
      die("ERROR: " . mysql_error());
    }	

  $data = array();
  
  while($row = mysqli_fetch_assoc($fetch_result)) {

    $success = false;  
    $new_failure = true;
    if($test_run_result = mysqli_query($db, sprintf("SELECT id FROM test_runs WHERE test_runs.server_id = %d ORDER BY id DESC LIMIT 1", $row['id']))) {
      if($test_run_row = mysqli_fetch_assoc($test_run_result)) {
        $latest_test_run = $test_run_row['id'];
        if($test_run_result = mysqli_query($db, sprintf("
SELECT  (SELECT count(id) FROM rendering_results WHERE rendering_results.test_run_id=%d) IS NOT NULL as rendering_exists,
          (SELECT success FROM rendering_results WHERE rendering_results.test_run_id=%d AND success='N'  LIMIT 1) IS NULL AS rendering_success,
        (SELECT count(id) FROM custom_results WHERE custom_results.test_run_id=%d) IS NOT NULL as custom_exists,
          (SELECT success FROM custom_results WHERE custom_results.test_run_id=%d AND success='N' LIMIT 1) IS NULL AS  custom_success,
        (SELECT count(id) FROM console_results WHERE console_results.test_run_id=%d) IS NOT NULL as console_exists,
          (SELECT success FROM console_results WHERE console_results.test_run_id=%d AND success='N' LIMIT 1) IS NULL AS console_success,
        (SELECT count(id) from error_results WHERE error_results.test_run_id=%d AND error_results.new_failure = 'Y') IS NOT NULL as error_exists,
          (SELECT COUNT(error_results.id) FROM error_results WHERE error_results.test_run_id=%d LIMIT 1 )=0 AS exec_success"
        , $latest_test_run, $latest_test_run, $latest_test_run, $latest_test_run, $latest_test_run, $latest_test_run, $latest_test_run, $latest_test_run))) {
          if($test_run_row = mysqli_fetch_assoc($test_run_result)) {
            $success = (!$test_run_row['rendering_exists'] or $test_run_row['rendering_success'])
                    && (!$test_run_row['custom_exists'] or $test_run_row['custom_success'] )
                    && (!$test_run_row['console_exists'] or $test_run_row['console_success'] )
                    && (!$test_run_row['error_exists'] or $test_run_row['exec_success'] );
          }
          $new_failure = true;
          if(!$success) {
            $new_failure_result = mysqli_query($db, sprintf("
            SELECT (SELECT new_failure FROM rendering_results WHERE rendering_results.test_run_id=%d AND success='N' AND new_failure='Y' LIMIT 1) IS NOT NULL AS rendering_fail_new,
                   (SELECT new_failure FROM custom_results WHERE custom_results.test_run_id=%d AND success='N' AND new_failure='Y' LIMIT 1) IS NOT NULL AS  custom_fail_new,
                   (SELECT new_failure FROM console_results WHERE console_results.test_run_id=%d AND success='N' AND new_failure='Y' LIMIT 1) IS NOT NULL AS console_fail_new,
                   (SELECT COUNT(error_results.id) FROM error_results WHERE error_results.test_run_id=%d AND new_failure='Y' LIMIT 1)>0 AS exec_fail_new
            ", $latest_test_run, $latest_test_run, $latest_test_run, $latest_test_run));
            if($new_fail_row = mysqli_fetch_assoc($new_failure_result)) {
              $new_failure = ($test_run_row['rendering_exists'] and $new_fail_row['rendering_fail_new'])
                          || ($test_run_row['custom_exists'] and $new_fail_row['custom_fail_new'])
                          || ($test_run_row['console_exists'] and $new_fail_row['console_fail_new'])
                          || ($test_run_row['error_exists'] and $new_fail_row['exec_fail_new']);
            }
          }          
        }
      } else { // No test runs for this server so mark it as not failed
        $success = true;
        $new_failure = false;      
      }	      
    }
    $server = array(
      "id" => $row['id'],
      "build_name" => $row['build_name'],
      "hardware_name" => $row['hardware_name'],
      "success" => $success,
      "new_failure" => $new_failure
    );
    if($server['hardware_name'] == null) {
      $server['hardware_name'] = "Unspecified";
    }
    
    
    array_push($data, $server);
  }
  return $data;
}


?>