<?php
require_once $_SERVER['DOCUMENT_ROOT'] . "/db.php";
require_once $_SERVER['DOCUMENT_ROOT'] . "/touch_report.php";

header('Content-Type: application/json');
$tr = new touch_report();
$last_object = $tr->most_recent_object();

if(!$last_object) {
    echo json_encode(false);
    exit;
}

echo json_encode($tr->object_information());
exit;

?>