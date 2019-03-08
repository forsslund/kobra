-- Run this file if your database is based on result_database_structure.sql from SVN revision 3657 (November 30 2016)
ALTER TABLE `console_results`
  ADD COLUMN `new_failure` enum('Y','N') after `diff`;
  
ALTER TABLE `custom_results`
  ADD COLUMN `new_failure` enum('Y','N') after `diff`;

ALTER TABLE `error_results`
  ADD COLUMN `new_failure` enum('Y','N') after `stderr`;
  
ALTER TABLE `rendering_results`
  ADD COLUMN `new_failure` enum('Y','N') after `diff_image`;


COLLATE='utf8_general_ci'
ENGINE=InnoDB
;