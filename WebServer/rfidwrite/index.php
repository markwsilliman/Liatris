<?php
/*
 * RFID reader reads a tag and passes it to this script.  This writes that value to the database.
 */

require_once $_SERVER['DOCUMENT_ROOT'] . "/db.php";

header('Content-Type: application/json');

if(!isset($_POST['rfid_api_guid']) || trim($_POST['rfid_api_guid']) == "") {
    echo json_encode(array("success" => false));
    return false;
}

$db = new db();

$guid = $_POST['rfid_api_guid'];

if($guid == "(No Tags)" || $guid == "") {
    echo json_encode(array("success" => false));
    return false;
}

if(strpos($guid,", Disc") !== false) {
    $guid = substr($guid,0,strpos($guid,", Disc"));
    $guid = str_replace("Tag:","",$guid);
    $guid = trim($guid);
    $guid = str_replace(" ","",$guid); //remove spaces
}

$guid = $db->cl($guid);

//prevent duplicates; every object should have it's own unique GUID.  In the future it'd be better to move to MIT's EPC format (combining both an EPC_ID & TAG_ID allowing anti-duplication to be testing against TAG_ID and not EPC_ID therefore allowing unlimited of the same type of objects)
$result = $db->query("select * from rfid where guid = '$guid' limit 1");
if($row = $db->fetch_array($result)) {
    echo json_encode(array("success" => false));
    return false;
}

$db->query("insert into rfid values(NULL,'$guid',NOW())");
echo json_encode(array("success" => true));