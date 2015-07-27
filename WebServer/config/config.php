<?php

/*
 * Config file.  Modify the following for your local mysql database.
 */

class config {
    //val
    public static function val($key) {
        $a = array();
        $a["db_ip"] = "localhost"; //IP of database
        $a["db_user"] = "root"; //username of database
        $a["db_pass"] = "turtlebot"; //password of database

        if(!array_key_exists($key,$a)) {
            dies("config static non-existent key [" . htmlspecialchars($key) . "]");
        }

        return $a[$key];
    }
    //end val
}