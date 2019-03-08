

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET NAMES utf8 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;

-- Dumping database structure for testdatabase
CREATE DATABASE IF NOT EXISTS `testdatabase` /*!40100 DEFAULT CHARACTER SET utf8 */;
USE `testdatabase`;


-- Dumping structure for table testdatabase.console_results
CREATE TABLE IF NOT EXISTS `console_results` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `test_run_id` int(10) unsigned NOT NULL,
  `file_id` int(10) unsigned NOT NULL,
  `case_id` int(10) unsigned NOT NULL,
  `step_id` int(10) unsigned NOT NULL,
  `success` enum('Y','N') NOT NULL,
  `output` text NOT NULL,
  `baseline` text,
  `diff` text,
  `new_failure` enum('Y','N'),
  `error_type` enum('NO_ERROR','STEP_NOT_FINISHED', 'NO_OUTPUT', 'NO_BASELINE', 'CONTENT_MISMATCH') NULL DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `test_run_id_success` (`test_run_id`,`success`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;


-- Dumping structure for table testdatabase.custom_results
CREATE TABLE IF NOT EXISTS `custom_results` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `test_run_id` int(10) unsigned NOT NULL,
  `file_id` int(10) unsigned NOT NULL,
  `case_id` int(10) unsigned NOT NULL,
  `step_id` int(10) unsigned NOT NULL,
  `success` enum('Y','N') NOT NULL,
  `output` mediumtext,
  `baseline` text,
  `diff` text,
  `new_failure` enum('Y','N'),
  `error_type` enum('NO_ERROR', 'STEP_NOT_FINISHED', 'NO_OUTPUT', 'NO_BASELINE', 'CONTENT_MISMATCH') NULL DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `test_run_id_success` (`test_run_id`,`success`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8 ROW_FORMAT=COMPACT;


-- Dumping structure for table testdatabase.error_results
CREATE TABLE IF NOT EXISTS `error_results` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `test_run_id` int(10) unsigned NOT NULL,
  `file_id` int(10) unsigned NOT NULL,
  `case_id` int(10) unsigned NOT NULL,
  `step_id` int(10) unsigned NOT NULL,
  `stdout` text,
  `stderr` text,
  `new_failure` enum('Y','N'),
  `error_type` enum('CASE_STDERR', 'CASE_NOT_STARTED', 'CASE_NOT_FINISHED', 'STEP_NOT_RUN') NULL DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `test_run_id` (`test_run_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;



-- Dumping structure for table testdatabase.performance_results
CREATE TABLE IF NOT EXISTS `performance_results` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `test_run_id` int(10) unsigned NOT NULL,
  `file_id` int(10) unsigned NOT NULL,
  `case_id` int(10) unsigned NOT NULL,
  `step_id` int(10) unsigned NOT NULL,
  `full_profiling_data` text NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;



-- Dumping structure for table testdatabase.performance_result_data
CREATE TABLE IF NOT EXISTS `performance_result_data` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `performance_result_id` int(10) unsigned NOT NULL,
  `level` int(10) unsigned NOT NULL,
  `identifier` tinytext NOT NULL,
  `mean` float NOT NULL,
  `percent` float NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;


-- Dumping structure for table testdatabase.rendering_baselines
CREATE TABLE IF NOT EXISTS `rendering_baselines` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `file_id` int(10) unsigned NOT NULL,
  `case_id` int(10) unsigned NOT NULL,
  `step_id` int(10) unsigned NOT NULL,
  `timestamp` datetime DEFAULT NULL,
  `image` mediumblob NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;


-- Dumping structure for table testdatabase.rendering_results
CREATE TABLE IF NOT EXISTS `rendering_results` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `test_run_id` int(10) unsigned NOT NULL,
  `file_id` int(10) unsigned NOT NULL,
  `case_id` int(10) unsigned NOT NULL,
  `step_id` int(10) unsigned NOT NULL,
  `success` enum('Y','N') NOT NULL,
  `output_image` mediumblob,
  `diff_image` mediumblob,
  `new_failure` enum('Y','N'),
  `error_type` enum('NO_ERROR','STEP_NOT_FINISHED','NO_OUTPUT','NO_BASELINE','CONTENT_MISMATCH','SIZE_MISMATCH') DEFAULT NULL,
  `diff_pixels` int(11) DEFAULT NULL,
  `threshold` int(11) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `test_run_id_success` (`test_run_id`,`success`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8 ROW_FORMAT=COMPACT;



-- Dumping structure for table testdatabase.servers
CREATE TABLE IF NOT EXISTS `servers` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `build_name` tinytext NOT NULL,
  `hardware_name` tinytext,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;



-- Dumping structure for table testdatabase.test_cases
CREATE TABLE IF NOT EXISTS `test_cases` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `case_name` text NOT NULL,
  `description` mediumtext,
  `svn_url_x3d` text,
  `svn_url_script` text,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;




-- Dumping structure for table testdatabase.test_categories
CREATE TABLE IF NOT EXISTS `test_categories` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `path` mediumtext NOT NULL,
  `description` mediumtext NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;



-- Dumping structure for table testdatabase.test_files
CREATE TABLE IF NOT EXISTS `test_files` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `filename` text NOT NULL,
  `description` mediumtext,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;



-- Dumping structure for table testdatabase.test_runs
CREATE TABLE IF NOT EXISTS `test_runs` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `timestamp` datetime NOT NULL,
  `server_id` int(10) unsigned NOT NULL,
  `allow_deletion` enum('Y','N') NOT NULL DEFAULT 'Y',
  `description` tinytext,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;



-- Dumping structure for table testdatabase.test_steps
CREATE TABLE IF NOT EXISTS `test_steps` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `step_name` tinytext NOT NULL,
  `test_case_id` int(10) unsigned NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

/*!40101 SET SQL_MODE=IFNULL(@OLD_SQL_MODE, '') */;
/*!40014 SET FOREIGN_KEY_CHECKS=IF(@OLD_FOREIGN_KEY_CHECKS IS NULL, 1, @OLD_FOREIGN_KEY_CHECKS) */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
