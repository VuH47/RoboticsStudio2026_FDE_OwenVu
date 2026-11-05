#!/usr/bin/env python3

import sqlite3
from datetime import datetime
from pathlib import Path


class MissionDatabase:
    """
    Database for the FDE alpine hazard inspection system.

    Structure:

        Hazard
        ├── Drone evidence
        └── Optional Husky mission
            ├── Ground inspection evidence
            └── Post-clearance evidence
    """

    def __init__(self, storage_dir=None):

        if storage_dir is None:
            storage_dir = Path.home() / "fde_missions"

        self.storage_dir = Path(storage_dir)
        self.storage_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.database_path = (
            self.storage_dir / "missions.db"
        )

        self.evidence_dir = (
            self.storage_dir / "evidence"
        )

        self.evidence_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.connection = sqlite3.connect(
            self.database_path
        )

        self.connection.row_factory = sqlite3.Row

        self.connection.execute(
            "PRAGMA foreign_keys = ON"
        )

        self._create_tables()

    # =============================================================
    # CREATE TABLES
    # =============================================================

    def _create_tables(self):

        cursor = self.connection.cursor()

        # ---------------------------------------------------------
        # HAZARDS
        # ---------------------------------------------------------

        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS hazards (
                hazard_id TEXT PRIMARY KEY,

                type TEXT NOT NULL,
                status TEXT NOT NULL,

                detection_confidence REAL,
                husky_solvable REAL,

                detected_by TEXT,
                time_detected TEXT,

                x REAL,
                y REAL,
                z REAL,

                notes TEXT
            )
            """
        )

        # ---------------------------------------------------------
        # HUSKY MISSIONS
        # ---------------------------------------------------------

        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS husky_missions (
                husky_mission_id INTEGER PRIMARY KEY AUTOINCREMENT,

                hazard_id TEXT NOT NULL UNIQUE,

                status TEXT NOT NULL,

                assigned_at TEXT,
                started_at TEXT,
                completed_at TEXT,

                resolution TEXT,

                FOREIGN KEY (hazard_id)
                    REFERENCES hazards(hazard_id)
                    ON DELETE CASCADE
            )
            """
        )

        # ---------------------------------------------------------
        # EVIDENCE
        # ---------------------------------------------------------

        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS evidence (
                evidence_id INTEGER PRIMARY KEY AUTOINCREMENT,

                hazard_id TEXT NOT NULL,

                evidence_type TEXT NOT NULL,
                file_path TEXT NOT NULL,
                captured_at TEXT NOT NULL,

                FOREIGN KEY (hazard_id)
                    REFERENCES hazards(hazard_id)
                    ON DELETE CASCADE
            )
            """
        )

        self.connection.commit()

    # =============================================================
    # CREATE HAZARD
    # =============================================================

    def create_hazard(
        self,
        hazard_id,
        hazard_type,
        status,
        x,
        y,
        z,
        detection_confidence=None,
        husky_solvable=None,
        detected_by="Drone",
        time_detected=None,
        notes=None,
    ):

        if time_detected is None:
            time_detected = datetime.now().isoformat(
                timespec="seconds"
            )

        cursor = self.connection.cursor()

        cursor.execute(
            """
            INSERT INTO hazards (
                hazard_id,
                type,
                status,
                detection_confidence,
                husky_solvable,
                detected_by,
                time_detected,
                x,
                y,
                z,
                notes
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                hazard_id,
                hazard_type,
                status,
                detection_confidence,
                husky_solvable,
                detected_by,
                time_detected,
                x,
                y,
                z,
                notes,
            ),
        )

        # Create evidence folder for this hazard.

        hazard_evidence_dir = (
            self.evidence_dir / hazard_id
        )

        hazard_evidence_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.connection.commit()

        print(
            f"[MissionDatabase] "
            f"Created hazard: {hazard_id}"
        )

    # =============================================================
    # UPDATE HAZARD STATUS
    # =============================================================



    # =============================================================
    # UPDATE LIVE HAZARD DETECTION
    # =============================================================

    def update_hazard_detection(
        self,
        hazard_id,
        status,
        x,
        y,
        z,
    ):
        """Update a tracked hazard's status and coordinates."""

        self.connection.execute(
            """
            UPDATE hazards
            SET
                status = ?,
                x = ?,
                y = ?,
                z = ?
            WHERE hazard_id = ?
            """,
            (
                status,
                float(x),
                float(y),
                float(z),
                hazard_id,
            ),
        )

        self.connection.commit()


    def update_hazard_status(
        self,
        hazard_id,
        status,
    ):

        cursor = self.connection.cursor()

        cursor.execute(
            """
            UPDATE hazards
            SET status = ?
            WHERE hazard_id = ?
            """,
            (
                status,
                hazard_id,
            ),
        )

        self.connection.commit()

    # =============================================================
    # UPDATE HAZARD NOTES
    # =============================================================

    def update_hazard_notes(
        self,
        hazard_id,
        notes,
    ):

        cursor = self.connection.cursor()

        cursor.execute(
            """
            UPDATE hazards
            SET notes = ?
            WHERE hazard_id = ?
            """,
            (
                notes,
                hazard_id,
            ),
        )

        self.connection.commit()

    # =============================================================
    # CREATE HUSKY MISSION
    # =============================================================

    def create_husky_mission(
        self,
        hazard_id,
    ):

        assigned_at = datetime.now().isoformat(
            timespec="seconds"
        )

        cursor = self.connection.cursor()

        cursor.execute(
            """
            INSERT INTO husky_missions (
                hazard_id,
                status,
                assigned_at,
                started_at,
                completed_at,
                resolution
            )
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            (
                hazard_id,
                "Queued",
                assigned_at,
                None,
                None,
                None,
            ),
        )

        husky_mission_id = cursor.lastrowid

        self.connection.commit()

        print(
            f"[MissionDatabase] "
            f"Created Husky mission "
            f"HM-{husky_mission_id:03d} "
            f"for {hazard_id}"
        )

        return husky_mission_id

    # =============================================================
    # START HUSKY MISSION
    # =============================================================

    def start_husky_mission(
        self,
        husky_mission_id,
    ):

        started_at = datetime.now().isoformat(
            timespec="seconds"
        )

        cursor = self.connection.cursor()

        cursor.execute(
            """
            UPDATE husky_missions
            SET
                status = ?,
                started_at = ?
            WHERE husky_mission_id = ?
            """,
            (
                "In Progress",
                started_at,
                husky_mission_id,
            ),
        )

        self.connection.commit()

    # =============================================================
    # COMPLETE HUSKY MISSION
    # =============================================================

    def complete_husky_mission(
        self,
        husky_mission_id,
        resolution=None,
    ):

        completed_at = datetime.now().isoformat(
            timespec="seconds"
        )

        cursor = self.connection.cursor()

        cursor.execute(
            """
            UPDATE husky_missions
            SET
                status = ?,
                completed_at = ?,
                resolution = ?
            WHERE husky_mission_id = ?
            """,
            (
                "Completed",
                completed_at,
                resolution,
                husky_mission_id,
            ),
        )

        self.connection.commit()

    # =============================================================
    # CANCEL HUSKY MISSION
    # =============================================================

    def cancel_husky_mission(
        self,
        hazard_id,
    ):

        cursor = self.connection.cursor()

        cursor.execute(
            """
            DELETE FROM husky_missions
            WHERE hazard_id = ?
            """,
            (
                hazard_id,
            ),
        )

        self.connection.commit()

    # =============================================================
    # ADD EVIDENCE
    # =============================================================

    def add_evidence(
        self,
        hazard_id,
        evidence_type,
        file_path,
        captured_at=None,
    ):

        if captured_at is None:
            captured_at = datetime.now().isoformat(
                timespec="seconds"
            )

        cursor = self.connection.cursor()

        cursor.execute(
            """
            INSERT INTO evidence (
                hazard_id,
                evidence_type,
                file_path,
                captured_at
            )
            VALUES (?, ?, ?, ?)
            """,
            (
                hazard_id,
                evidence_type,
                str(file_path),
                captured_at,
            ),
        )

        evidence_id = cursor.lastrowid

        self.connection.commit()

        return evidence_id

    # =============================================================
    # GET ONE HAZARD
    # =============================================================

    def get_hazard(
        self,
        hazard_id,
    ):

        cursor = self.connection.cursor()

        cursor.execute(
            """
            SELECT *
            FROM hazards
            WHERE hazard_id = ?
            """,
            (
                hazard_id,
            ),
        )

        return cursor.fetchone()

    # =============================================================
    # GET ALL HAZARDS
    # =============================================================

    def get_hazards(self):

        cursor = self.connection.cursor()

        cursor.execute(
            """
            SELECT *
            FROM hazards
            ORDER BY time_detected DESC
            """
        )

        return cursor.fetchall()

    # =============================================================
    # GET HUSKY MISSION
    # =============================================================

    def get_husky_mission(
        self,
        hazard_id,
    ):

        cursor = self.connection.cursor()

        cursor.execute(
            """
            SELECT *
            FROM husky_missions
            WHERE hazard_id = ?
            LIMIT 1
            """,
            (
                hazard_id,
            ),
        )

        return cursor.fetchone()

    # =============================================================
    # GET ALL HUSKY MISSIONS
    # =============================================================

    def get_husky_missions(self):

        cursor = self.connection.cursor()

        cursor.execute(
            """
            SELECT *
            FROM husky_missions
            ORDER BY assigned_at DESC
            """
        )

        return cursor.fetchall()

    # =============================================================
    # GET EVIDENCE
    # =============================================================

    def get_evidence(
        self,
        hazard_id,
        evidence_type=None,
    ):

        cursor = self.connection.cursor()

        if evidence_type is None:

            cursor.execute(
                """
                SELECT *
                FROM evidence
                WHERE hazard_id = ?
                ORDER BY captured_at ASC
                """,
                (
                    hazard_id,
                ),
            )

        else:

            cursor.execute(
                """
                SELECT *
                FROM evidence
                WHERE hazard_id = ?
                  AND evidence_type = ?
                ORDER BY captured_at ASC
                """,
                (
                    hazard_id,
                    evidence_type,
                ),
            )

        return cursor.fetchall()

    # =============================================================
    # CLOSE
    # =============================================================

    def close(self):

        self.connection.close()