<?php
/*
 * Show log of most recent 100 RFID values.
 */

require_once $_SERVER['DOCUMENT_ROOT'] . "/db.php";

if(isset($_GET['rfid_report_print'])) {
    $rfid_report = new rfid_report();
    $rfid_report->print_report();
}

class rfid_report {
    public function print_report() {
        $db = new db();
        $result = $db->query("select * from rfid order by ID desc limit 100");
        echo "<table border='1'><tr><th>ID</th><th>GUID</th><th>DATE</th></tr>";
        while($row = $db->fetch_array($result)) {
            echo "<tr><td>" . $row['ID'] . "</td><td>" . htmlspecialchars($row['guid']) . "</td><td>" . $row['date_created'] . "</td></tr>";
        }
        echo "</table>";
    }
}