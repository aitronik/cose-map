-- MariaDB dump 10.19  Distrib 10.6.14-MariaDB, for debian-linux-gnu (x86_64)
--
-- Host: localhost    Database: semanticDatabase
-- ------------------------------------------------------
-- Server version	10.6.14-MariaDB-1:10.6.14+maria~ubu2004

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `classes`
--

DROP TABLE IF EXISTS `classes`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `classes` (
  `classId` int(11) NOT NULL,
  `name` varchar(100) NOT NULL,
  PRIMARY KEY (`classId`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `classes`
--

LOCK TABLES `classes` WRITE;
/*!40000 ALTER TABLE `classes` DISABLE KEYS */;
/*!40000 ALTER TABLE `classes` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `objects`
--

DROP TABLE IF EXISTS `objects`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `objects` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `classId` int(11) NOT NULL,
  `confidence` float NOT NULL,
  `colorR` int(10) unsigned NOT NULL,
  `colorG` int(10) unsigned NOT NULL,
  `colorB` int(10) unsigned NOT NULL,
  `centroidX` float NOT NULL,
  `centroidY` float NOT NULL,
  `centroidZ` float NOT NULL,
  `bbox_minX` float NOT NULL,
  `bbox_minY` float NOT NULL,
  `bbox_minZ` float NOT NULL,
  `bbox_maxX` float NOT NULL,
  `bbox_maxY` float NOT NULL,
  `bbox_maxZ` float NOT NULL,
  PRIMARY KEY (`id`),
  KEY `objects_FK` (`classId`),
  CONSTRAINT `objects_FK` FOREIGN KEY (`classId`) REFERENCES `classes` (`classId`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `objects`
--

LOCK TABLES `objects` WRITE;
/*!40000 ALTER TABLE `objects` DISABLE KEYS */;
/*!40000 ALTER TABLE `objects` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `voxels`
--

DROP TABLE IF EXISTS `voxels`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `voxels` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `classId` int(11) NOT NULL,
  `confidence` float NOT NULL,
  `colorR` int(10) unsigned NOT NULL,
  `colorG` int(10) unsigned NOT NULL,
  `colorB` int(10) unsigned NOT NULL,
  `positionX` float NOT NULL,
  `positionY` float NOT NULL,
  `positionZ` float NOT NULL,
  `volume` float NOT NULL,
  `objectId` int(11) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `voxels_FK` (`classId`),
  KEY `voxels_FK_1` (`objectId`),
  CONSTRAINT `voxels_FK` FOREIGN KEY (`classId`) REFERENCES `classes` (`classId`),
  CONSTRAINT `voxels_FK_1` FOREIGN KEY (`objectId`) REFERENCES `objects` (`id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `voxels`
--

LOCK TABLES `voxels` WRITE;
/*!40000 ALTER TABLE `voxels` DISABLE KEYS */;
/*!40000 ALTER TABLE `voxels` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Dumping routines for database 'semanticDatabase'
--
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2023-06-21 14:20:26
