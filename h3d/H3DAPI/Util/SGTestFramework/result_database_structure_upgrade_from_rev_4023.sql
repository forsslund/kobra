-- Run this file if your database is based on result_database_structure.sql from SVN revision 4023 (June 2 2017)
ALTER TABLE `console_results`
  ADD COLUMN `error_type` enum('NO_ERROR','STEP_NOT_FINISHED', 'NO_OUTPUT', 'NO_BASELINE', 'CONTENT_MISMATCH') NULL DEFAULT NULL;
  
ALTER TABLE `custom_results`
  ADD COLUMN `error_type` enum('NO_ERROR', 'STEP_NOT_FINISHED', 'NO_OUTPUT', 'NO_BASELINE', 'CONTENT_MISMATCH') NULL DEFAULT NULL;

ALTER TABLE `error_results`
  ADD COLUMN `error_type` enum('CASE_STDERR', 'CASE_NOT_STARTED', 'CASE_NOT_FINISHED', 'STEP_NOT_RUN') NULL DEFAULT NULL;
  
ALTER TABLE `rendering_results`
  ADD COLUMN `error_type` enum('NO_ERROR', 'STEP_NOT_FINISHED', 'NO_OUTPUT', 'NO_BASELINE', 'CONTENT_MISMATCH', 'SIZE_MISMATCH') NULL DEFAULT NULL,
	ADD COLUMN `diff_pixels` INT NULL DEFAULT NULL AFTER `error_type`,
	ADD COLUMN `threshold` INT NULL DEFAULT NULL AFTER `diff_pixels`;