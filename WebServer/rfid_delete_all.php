<?php
/*
 * Delete all RFID records.  This is just a convenient script to reset the RFID database between test runs.
 */

require_once $_SERVER['DOCUMENT_ROOT'] . "/db.php";

$db = new db();
$result = $db->query("delete from rfid");
echo "all RFID values deleted";