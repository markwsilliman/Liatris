<?php

require_once $_SERVER['DOCUMENT_ROOT'] . "/config/config.php";

/*
 * Database connection; Presuming your config file settings are valid it'll automatically create the database & tables if they don't exist.
 */

class db {
    /** @var mysqli**/
    private $conn;

    //__construct
    public function __construct() {
        $this->conn = mysqli_connect(config::val("db_ip"),config::val("db_user"),config::val("db_pass")) or die("mysqli_connect 1");
        $this->create_database();
        $this->conn->select_db("smarttable");
        $this->create_tables();
    }
    //end __construct

    //create_database
    private function create_database() {
        $query = "CREATE DATABASE IF NOT EXISTS smarttable";
        $this->query($query);
    }
    //end create_database

    //nc
    public function nc($val) {
        if(!is_numeric($val)) {
            die("nc [" . htmlspecialchars($val) . "]");
        }
        return $val;
    }
    //end nc

    //cl
    public function cl($val) {
        return $this->conn->escape_string($val);
    }
    //end cl

    //create_tables
    private function create_tables() {


        $query = "CREATE TABLE IF NOT EXISTS touch (
                    ID BIGINT AUTO_INCREMENT,
                    x_percent DOUBLE,
                    y_percent DOUBLE,
                    type_percent TINYINT,
                    date_created DATETIME,
                    PRIMARY KEY  (ID)
                )";
        $this->query($query);

        $query = "CREATE TABLE IF NOT EXISTS rfid (
                    ID BIGINT AUTO_INCREMENT,
                    guid VARCHAR(100),
                    date_created DATETIME,
                    PRIMARY KEY  (ID)
                )";
        $this->query($query);
    }
    //end create_tables

    //query
    public function query($sql) {
        $result = $this->conn->query($sql) or die("mysqli query 1 [" . htmlspecialchars($sql) . "]");
        return $result;
    }
    //end query

    //fetch_array
    public function fetch_array($result) {
        return $result->fetch_array(MYSQLI_ASSOC);
    }
    //end fetch_array


}