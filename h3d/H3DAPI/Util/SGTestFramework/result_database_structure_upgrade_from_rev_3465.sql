-- Run this file if your database is based on result_database_structure.sql from SVN revision 3465 (June 21st 2016)
ALTER TABLE `rendering_baselines`
	ADD COLUMN `timestamp` DATETIME NULL DEFAULT "2015-01-01" AFTER `step_id`;

ALTER TABLE `test_cases`
	ADD COLUMN `description` MEDIUMTEXT NULL AFTER `case_name`,
	ADD COLUMN `svn_url_x3d` TEXT NULL AFTER `description`,
	ADD COLUMN `svn_url_script` TEXT NULL AFTER `svn_url_x3d`;

ALTER TABLE `test_files`
	ADD COLUMN `description` MEDIUMTEXT NULL AFTER `filename`;

CREATE TABLE `test_categories` (
	`id` INT(11) NOT NULL AUTO_INCREMENT,
	`path` MEDIUMTEXT NOT NULL,
	`description` MEDIUMTEXT NOT NULL,
	PRIMARY KEY (`id`)
)

COLLATE='utf8_general_ci'
ENGINE=InnoDB
;