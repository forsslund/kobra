-- Run this file if your database is based on result_database_structure.sql from SVN revision 4164 (August 7 2017)


ALTER TABLE `test_runs`
	ADD COLUMN `allow_deletion` ENUM('Y','N') NOT NULL DEFAULT 'Y' AFTER `server_id`,
	ADD COLUMN `description` TINYTEXT NULL DEFAULT NULL;


ALTER TABLE `servers`
	CHANGE COLUMN `server_name` `build_name` TINYTEXT NOT NULL AFTER `id`,
	ADD COLUMN `hardware_name` TINYTEXT NOT NULL;