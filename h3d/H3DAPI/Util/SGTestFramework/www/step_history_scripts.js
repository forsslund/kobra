
function LoadSQLModel(test_case_id, server_id, result_callback) {
  // The model is a tree implemented with nested arrays
  // Every node is a category and every leaf is a test file.
  //
  // The node members are:
  // name : string
  // children : an array of other nodes
  //
  // The leaf (ie. test file) members are:
  // name : the test filename
  // testcases : an array of testcases
  // 
  // The testcases array has one entry for every testcase in the test file
  // The testcase members are:
  // name : the testcase name
  // time : the timestamp of the test run
  // server_id : the id of the server that ran this test
  // server_name : the name of the server that ran this test
  // min_fps : float
  // avg_fps : float
  // max_fps : float
  // history : an array containing history data from other runs of the same test
  //
  // The history array contains one entry for every previous run of this testcase on every test server sorted by their timestamp
  // The history object members are:
  // time : the timestamp of the testrun
  // server_id : the id of the server that ran this test
  // server_name : the name of the server that ran this test
  // min_fps : float
  // avg_fps : float
  // max_fps : float
  
  // Connect database
  var url = 'get_step_history.php?test_case_id=' + test_case_id + '&server_id=' + server_id;
  url += '&get_perf=true';

  $.ajax({
      type: 'GET',
      url: url,
      dataType: 'json',
      success:result_callback,
      error: function() {
      },
      async: true
  });
}



var max_display_count = location.hash.substr(location.hash.indexOf('max_display_count=')).split('&')[0].split('=')[1];
var param_server_id = location.hash.substr(location.hash.indexOf('server_id=')).split('&')[0].split('=')[1];
var param_case_id = location.hash.substr(location.hash.indexOf('case_id=')).split('&')[0].split('=')[1];

function getImageBlobURL(id, image_type, download_name) {
  var container = $('<div>');
  var link = $("<a>");
  link.addClass("image_download_link");
  link.attr("href",  "get_image.php?type="+image_type+"&id="+id+"&name="+download_name);
  link.attr("target",  "get_image.php?type="+image_type+"&id="+id+"&name="+download_name);
  link.attr("download", download_name);
  link.append("(Download)");
  var img = $("<img>");
  img.attr("src", "get_image.php?type="+image_type+"&id="+id+"&name="+download_name);
  img.attr("alt", image_type);
  img.addClass("TestResult_image");
  container.append(img);
  container.append(link);
  
  return container;
}

function attachImageViewer ( image_container, testcase, diff_container, download_name, diff_download_name, active_image ) {
  image_container.css( "cursor", "pointer" );
  image_container.click( function(){
    openImageViewer (
      testcase, 
      active_image,
      diff_container, 
      download_name, 
      diff_download_name );
    });
}

function generateGraph(div) {
  $('.TestResult_graph', div).remove();
  $('.TestResult_data', div).remove();
  var model = div.data('model');
  var graph_data = generateDatasets(model);

  var graph_div = $('<div>');
  graph_div.addClass('TestResult_graph');
  
  
  if(lowest_time == highest_time) {
    highest_time = (new Date(lowest_time).getTime()+1)
  }
  highest_time = (new Date(highest_time).getTime())
  lowest_time = (new Date(lowest_time).getTime())
  
   
  var data_div = $('<div>');
  data_div.addClass('TestResult_data');
  var data_list = $('<ul>');
  data_list.addClass('TestResult_data_list');
  data_list.append('Latest:');
  data_div.append(data_list);
  div.append(data_div);
  
   
  var graph_options = {
    series: {
      lines: {
        show: true,
        fill: true,
        steps: false,
      },
      points: {
        show: true
      }
    },
    colors: ["#7AD65C", "#5CD0D6", "#765CD6", "#D25CD6", "#D65C5C", "#D68F5C", "#D65C7E"],
    grid:  { hoverable: true }, //important! flot.tooltip requires this
    tooltip: {
      show: true,
      content: function(label, xval, yval, flotItem) {
        var total = 0;//parseFloat(yval);
        var res = flotItem.series.build_name + "<br/>" + label + ": " + getSelectedProperty(flotItem.series.data[flotItem.dataIndex][2]) + getSelectedUnit() + "<br/>";
        if(flotItem.series.prof_data.hasOwnProperty("children")) {
          for(var i = 0; i < flotItem.series.prof_data.children.length; ++i) {
            for(var d = 0; d < flotItem.series.prof_data.children[i].data.length; ++d) {
              if(flotItem.series.prof_data.children[i].data[d].timestamp.replace(' ', 'T') == flotItem.series.data[flotItem.dataIndex][2].timestamp.replace(' ', 'T')) {              
                var val = parseFloat(getSelectedProperty(flotItem.series.prof_data.children[i].data[d]));
                total += val;
                res += flotItem.series.prof_data.children[i].id + ": " + val + getSelectedUnit() +"<br/>";
                break;
              }
            }
          }
        }
        res += "stack: " + flotItem.series.stack;
        return res;
      }
    },
    xaxis: {
      mode: "time",
      min: lowest_time,
      max: highest_time,
      zoomRange: [(highest_time-lowest_time)/2, highest_time-lowest_time],
      panRange: [lowest_time, highest_time],
      tickFormatter: function (val, axis) {
          if(val) {
          var d = new Date(val);
          return d.toISOString().split("T")[0];
          }
          else return "NaN"
      }
    },
    yaxis: {
      min: 0,
      panRange: [0, (display_options.properties.selected == 'percent') ? 100 : null],
      zoomRange: [0, (display_options.properties.selected == 'percent') ? 100 : null],
    },
    zoom: {
      interactive: true
    },
    pan: {
      interactive: true
    },    
    legend: {
      show: true,
      container: data_list,
      sorted: false,
      labelFormatter: function(label, item) {
        if ((item.build_name == display_options.servers.current.build) && (item.hardware_name == display_options.servers.current.hardware))
          return label + " - " + getSelectedProperty(item.data[item.data.length-1][2]) + getSelectedUnit();
        else
          return null;
      }
    }
  }
		
  $('body').append(graph_div);


  // Options can be set globally. 
 
  $.plot(graph_div, graph_data, graph_options);
  
  data_div.detach();
  div.append(graph_div); 
  div.append(data_div);

           
}

function generateImages(div) {
  var testcase = $(div).data('model');
  var container = $('<div>');
  container.addClass('TestResult_rendering');

  var diff_container = $('<div>');
  diff_container.addClass('diff_container');
  container.append(diff_container)

  var download_name = testcase.name + "_" + testcase.step_name + ".png";
  var diff_download_name =  "diff_" + download_name;
  // If it succeded then show the baseline image
  // If it failed and there's no baseline then show the output and complain about the lack of a baseline
  // If it failed and there is a baseline then show the baseline, the diff and the output
  if(testcase.success == 'Y') {
    var succeeded = $("<span>");
    succeeded.addClass('test_successful');
    succeeded.append("Step successful!");
    div.append(succeeded);

    
    if (testcase.diff_pixels != null && testcase.threshold != null) {
        var span = $("<span>(" + testcase.diff_pixels + " differing pixels with a threshold of " + testcase.threshold + ")</span>");
        container.append(span);
        container.append("<br />");
    }

    var image_container = $('<div>');
    if(testcase.baseline_id != "") {
      var image_container = $('<div>');
      image_container.addClass('TestResult_image_div'); 
      image_container.append("Baseline:</br>");
      image_container.append(getImageBlobURL(testcase.baseline_id, "baseline", download_name));
      diff_container.append(image_container);
    }
  } else { // Didn't succeed
    var succeeded = $("<span>");
    succeeded.addClass('test_failed');
    if(testcase.new_failure == 'Y') {
      succeeded.addClass('test_failed_new');
    }
    div.append(succeeded);

    if (testcase.diff_pixels != null && testcase.threshold != null) {
        var span = $("<span>(" + testcase.diff_pixels + " differing pixels with a threshold of " + testcase.threshold + ")</span>");
        container.append(span);
        container.append("<br />");
    }

    // As a fallback for making sure no important information is hidden for old test cases, we set error_type to CONTENT_MISMATCH if it is null
    // This  will make sure that content mismatch cases actually show their outputs and diffs, as CONTENT_MISMATCH is the error_type that prints all information
    if (testcase.error_type == null) {
        testcase.error_type = "CONTENT_MISMATCH";
    }

    if (testcase.error_type == "NO_BASELINE") {
        succeeded.append("Step failed - NO_BASELINE: This step is missing a baseline!");
    } else {
        if (testcase.baseline_id != "") {
            var image_container = $('<div>');
            image_container.addClass('TestResult_image_div');
            image_container.append("Baseline:</br>");
            image_container.append(getImageBlobURL(testcase.baseline_id, "baseline", download_name));
            diff_container.append(image_container);

            // add functionality to open diff viewer on clicking thumbnail
            attachImageViewer ( 
              image_container, testcase, diff_container, 
              download_name, diff_download_name, "baseline" );
        }
    }
    if (testcase.error_type == 'STEP_NOT_FINISHED') {
        succeeded.append("Step failed - STEP_NOT_FINISHED: Freeze, Crash or Python exception happened before step was done!")
    }
    else {
        if (testcase.error_type == "NO_OUTPUT") {
            succeeded.append("Step failed - NO_OUTPUT: Step didn't output image!");
        } else {
            var image_container = $('<div>');
            image_container.addClass('TestResult_image_div');
            image_container.append("Output:</br>");
            image_container.append(getImageBlobURL(testcase.id, "result", download_name));
            diff_container.append(image_container);

            // add functionality to open diff viewer on clicking thumbnail
            attachImageViewer ( 
              image_container, testcase, diff_container, 
              download_name, diff_download_name, "result" );

            if (testcase.error_type == "SIZE_MISMATCH") {
                succeeded.append("Step failed - SIZE_MISMATCH: Output image dimensions do not match baseline!")
            } else if (testcase.error_type == "CONTENT_MISMATCH") {
                succeeded.append("Step failed - CONTENT_MISMATCH: Image contents differ too much!");

                var image_container = $('<div>');
                image_container.addClass('TestResult_image_div');
                image_container.append("Diff:</br>");
                image_container.append(getImageBlobURL(testcase.id, "diff", diff_download_name));
                diff_container.append(image_container);

                // add functionality to open diff viewer on clicking thumbnail
                attachImageViewer ( 
                  image_container, testcase, diff_container, 
                  download_name, diff_download_name, "diff" );

                // Add baseline update checkbox here
                var baseline_update = $('<div>');
                baseline_update.addClass('baseline_update');
                container.append(baseline_update);

                var cb = $('<input>Include in baseline update</input>');
                cb.attr('type', 'checkbox');
                cb.addClass('baselineUpdateCheckbox');
                cb.data('model', testcase);
                baseline_update.append(cb);
            }
        }
    }

  }
  div.append(container);
}

function generateConsole(div) {
  var testcase = $(div).data('model');
  var container = $('<div>');
  var succeeded = $("<span>");
  div.append(succeeded);
  
  if(testcase.success == "N"){
    succeeded.addClass('test_failed');
    if(testcase.new_failure == 'Y') {
      succeeded.addClass('test_failed_new');
    }
  } else {
    succeeded.addClass('test_successful');
    succeeded.append("Step successful!");
  } 
  
  container.addClass('TestResult_console');
  var diff_container = $('<div>');
  diff_container.addClass('diff_container');
  container.append(diff_container);
  if (testcase.error_Type == 'STEP_NOT_FINISHED') {
      succeeded.append("Step failed - STEP_NOT_FINISHED: Freeze, Crash or Python exception happened before step was done!")
  } else {
      if (testcase.error_type == 'NO_OUTPUT') {
          succeeded.append("Step failed - NO_OUTPUT: This didn't output to console!");
      } else {
          var output_and_button = $('<div>');
          output_and_button.addClass('console_container_and_button');
          var output = $('<div>');
          output.addClass('stdout_div');
          output.append("<b style='text-transform:capitalize;'>" + testcase.result_type + ":</b></br></br>");
          output.append(testcase.text_output.split('\n').join('</br>'));
          output_and_button.append(output);
          diff_container.append(output_and_button);
      }
      if (testcase.success == "N") {
          if (testcase.error_type == 'NO_BASELINE') {
              succeeded.append("Step failed - NO_BASELINE: Test step is missing a baseline!");
          } else {
              succeeded.append("Step failed - " + testcase.error_type);
              if (testcase.hasOwnProperty("text_baseline")) {
                  var baseline_and_button = $('<div>');
                  baseline_and_button.addClass('console_container_and_button');
                  var baseline = $('<div>');
                  baseline.addClass('stdout_div');
                  baseline.append("<b>Baseline:</b></br></br>");
                  baseline.append(testcase.text_baseline.split('\n').join('</br>'));
                  baseline_and_button.append(baseline);
                  diff_container.append(baseline_and_button);
              }
              if (testcase.hasOwnProperty("text_diff")) {
                  var diff_and_button = $('<div>');
                  diff_and_button.addClass('console_container_and_button');
                  var diff = $('<div>');
                  diff.addClass('stdout_div');
                  diff.append("<b>Diff:</b></br></br>");
                  diff.append(testcase.text_diff.split('\n').join('</br>'));
                  diff_and_button.append(diff);
                  diff_container.append(diff_and_button);
              }
              if (testcase.hasOwnProperty("text_baseline") && testcase.hasOwnProperty("text_diff")) {
                  // Add baseline update checkbox here
                  var baseline_update = $('<div>');
                  baseline_update.addClass('baseline_update');
                  container.append(baseline_update);

                  var cb = $('<input>Include in baseline update</input>');
                  cb.attr('type', 'checkbox');
                  cb.addClass('baselineUpdateCheckbox');
                  cb.data('model', testcase);
                  baseline_update.append(cb);
              }
          }
      } else {
          $('.TestStep_name', div).addClass('minimized');
      }
  }
  
  div.append(container);
}

function generateError(div) {
  var testcase = $(div).data('model');
  var container = $('<div>');
  container.addClass('TestResult_error');
  
  var std = $('<div>');
  std.addClass('std_div');
  std.append(testcase.stdout.split('\n').join('</br>'));
  std.append("</br></br><b>stderr:</b></br></br>");
  std.append(testcase.stderr.split('\n').join('</br>'));
  container.append(std);
  
  if(testcase.new_failure == 'Y') {
    container.addClass('test_failed_new');
  } 
  
  div.append(container);
}


function ConstructTestCases(model, target, path) {
  var container = $('<div>');
  container.addClass('Test_Container');
  
  if (model.description) {
    var description = $("<div>");
    description.addClass('TestCase');
    description.addClass('description');
    description.append(model.description);
    container.append(description);
  }

  target.append(container);
  

  if(model.testcases.length > 0) {
    model.testcases.sort(function(a, b) {
    /* First check that it is the same case */
    var i = a.time.localeCompare(b.time);
    if (i == 0) { /* Same case, so return comparison of step name instead */
      return a.step_name.localeCompare(b.step_name);
    }
    else
      return a.time.localeCompare(b.time);
    }
    );
    var current_case_name = 'placeholder that shouldn\'t ever match';
    var current_step_name = 'another placeholder';
    for(var i = 0; i < model.testcases.length; i++) {
      if(model.testcases[i].time != current_case_name) {
        var case_div = $('<div>');
        case_div.addClass('TestCase');
        case_div.addClass('Category_Item');
        var case_name = $('<div>');
        case_name.addClass("TestResult_name");
        
        var case_name_time = $("<a>");
        case_name_time.append(model.testcases[i].time);
        case_name.append(case_name_time);

        //var p = $(target).parent();
        //while( $("input", p).length ){
        //  $("input", p).prop("checked", true);
        //  p = $(p).parent();
        //}

        if(model.testcases[i].success == 'Y') {
          case_name.addClass("test_successful");
        } else {
          case_name.addClass("test_failed");
          if(model.testcases[i].new_failure == 'Y') {
            case_name.addClass('test_failed_new');
          }
        }
        
        case_div.append(case_name);

        if (model.testcases[i].svn_url_x3d && model.testcases[i].svn_url_x3d != "") {
          var case_svn_x3d_link = $("<a>");
          case_svn_x3d_link.append("x3d");
          case_svn_x3d_link.attr('href', encodeURI(model.testcases[i].svn_url_x3d));
          case_div.append(case_svn_x3d_link);
          case_div.append(" ");
        }
        
        container.append(case_div);
        current_case_name = model.testcases[i].time;
      }
      
      if(model.testcases[i].success != 'Y') {
        case_name.removeClass("test_successful");
        case_name.addClass("test_failed");
        if(model.testcases[i].new_failure == 'Y') {
          case_name.addClass('test_failed_new');
        }
      }
      
      // This is specifically for suppressing the green label
//      if(model.testcases[i].result_type == 'error' && $(".TestResult", case_div).length == 0 && (i < model.testcases.length-1 && (model.testcases[i+1].name != current_case_name)) ) {
//        case_name.removeClass("test_successful");
//        case_name.addClass("test_failed");
 //     }
            
      var step_div = $('<div>');
      step_div.addClass('TestResult');
      if(current_step_name != model.testcases[i].step_name) {
        step_div.addClass('TestResult_first_in_step');
        current_step_name = model.testcases[i].step_name;
      }
      var name_div = $('<div>');
      name_div.addClass('TestStep_name');
      name_div.click(function(){ // onclick function for toggling the presence of a minimized-class
        $(this).toggleClass("minimized");
      });
        
      if(model.testcases[i].result_type == 'error') {
        if(model.testcases[i].step_name == '') {
          name_div.append("Testcase failed");
        } else {
          name_div.append(model.testcases[i].step_name);
        }
        name_div.addClass("test_failed");
        if(model.testcases[i].new_failure == 'Y') {
          name_div.addClass('test_failed_new');
        }
      } else {
        name_div.append("Step: " + model.testcases[i].step_name);
      }
      step_div.append(name_div);
      step_div.data('model', model.testcases[i]); // Store the associated testCase with the div
      
      if(model.testcases[i].result_type != "performance")
        case_div.append(step_div);
      
      // Skip performance data
      // if(!(model.testcases[i].result_type == "performance" && !model.testcases[i].hasOwnProperty("profiler_data")))
      //   case_div.append(step_div);
      // else {
      //   case_name.removeClass("test_successful");
      //   case_name.addClass("test_failed");
      //   container.removeClass("test_successful");
      //   container.addClass("test_failed");
      //   case_name.append(" - Missing performance data");
      // }
      name_div.toggleClass("minimized");
    }
    container.css({"display":"block"});
  }  
}

function ConstructList(model, target, path) {
  for (var i = 0; i < model.length; i++) {
    if(model[i]) {
      var ul = $('<ul>');
      target.append(ul);
      ul.attr('class', 'Category_Item');
      
      var label = $('<label>');
      label.attr('class', 'noselect');       
      var glyph = $('<div>');
      glyph.attr('class', 'glyph');
      glyph.html('▶');
        
      label.append(glyph);

      case_name = '';
      if( model[i].hasOwnProperty('testcases') && model[i].testcases.length > 0 ){
        case_name = model[i].testcases[0].name;
      }
      
      var name = $('<a><h3>'+model[i].name+ ' - ' + case_name+ '</h3></a>');
      name.data('category_name', path + model[i].name + "/");
      label.append(name);
      
      
      ul.append(label);
      
      if (model[i].hasOwnProperty('children')) {
        if (model[i].description) {
            var description_container = $('<ul>');
			description_container.addClass("Category_Item");
		    var description = $("<div>");
		    description.addClass('TestCase');
		    description.append(model[i].description);
            description_container.append(description);
		    ul.append(description_container);
        }
        ConstructList(model[i].children, ul, path + model[i].name + "/");
        if($('.test_failed', ul).length > 0) {
          name.addClass('test_failed');
          if($('.test_failed_new', ul).length > 0) {
            name.addClass("test_failed_new");
          }
        }         
        else
          name.addClass('test_successful');
      }
      else if (model[i].hasOwnProperty('testcases')) {
        if(model[i].success == undefined || (model[i].success))
          name.addClass('test_successful');
        else
          name.addClass('test_failed');
        ConstructTestCases(model[i], ul, path + model[i].name + "/");
        if($('.test_failed', ul).length > 0) {
          name.addClass('test_failed');
          if($('.test_failed_new', ul).length > 0) {
            name.addClass("test_failed_new");
          }
        }            
        else
          name.addClass('test_successful');        
        
        first = false;
      }
             
    }
  }      
}

LoadSQLModel(param_case_id, param_server_id, function(data) {
  $('#Step_List').empty();    
  $('#loading_spinner_container').hide();
  model = data;
  first = true;
  ConstructList(model, $('#Step_List'), "");
  $('.TestResult').each(function() {
  var testResult = $(this).data('model');
    // Performance display needs some extra work, 
    // but its rarely used so disabled for now
    //if(testResult.result_type=="performance") {
    //  generateGraph($(this));
    //} 
    if (testResult.result_type=="rendering") {
      generateImages($(this));
    } else if (testResult.result_type=="console") {
      generateConsole($(this));
    } else if (testResult.result_type=="custom") {
      generateConsole($(this));
    } else if (testResult.result_type=='error') {
      generateError($(this));
    }
  });
  
});