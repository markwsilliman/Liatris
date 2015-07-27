<?php
/*
 *  Save as config/config.php and modify with your local values
 */

class config {
    //val
    public static function val($key) {
        $a = array();
        $a["db_ip"] = ""; //IP of database
        $a["db_user"] = ""; //username of database
        $a["db_pass"] = ""; //password of database

        if(!array_key_exists($key,$a)) {
            dies("config static non-existent key [" . htmlspecialchars($key) . "]");
        }

        return $a[$key];
    }
    //end val
}