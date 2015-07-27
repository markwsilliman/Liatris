<?php
/*
 * Returns the last RFID value in JSON format
 */

require_once $_SERVER['DOCUMENT_ROOT'] . "/db.php";

header('Content-Type: application/json');

$db = new db();
$result = $db->query("select * from rfid where date_created >= SUBDATE(NOW(),INTERVAL 1 MINUTE) order by ID desc limit 1");
if($row = $db->fetch_array($result)) {
    echo json_encode($row['guid']);
}
else {
    echo json_encode(false);
}
exit;