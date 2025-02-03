import xml.etree.ElementTree as ET
import pytak
from datetime import datetime
import uuid
from typing import List, Tuple

def create_gps_COT(uuid: str, cot_type : str, latitude : str, longitude: str, altitude : str):
    """Create a CoT event based on the GPS data."""
    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", cot_type)
    root.set("uid", uuid)  # Use topic name as UID for identification
    root.set("how", "m-g")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(3600))

    pt_attr = {
        "lat": str(latitude),
        "lon": str(longitude),
        "hae": str(altitude),
        "ce": "10",
        "le": "10",
    }

    ET.SubElement(root, "point", attrib=pt_attr)

    return ET.tostring(root, encoding="utf-8")

def create_casevac_COT(uuid, casualty_id, gps, zap_num, mechanism, injury, signs_symptoms, treatments, physician_name):
    # Create root event element
    event = ET.Element(
        "event",
        {
            "version": "2.0",
            "uid": casualty_id,
            "type": "b-r-f-h-c",
            "how": "h-g-i-g-o",
            "time": pytak.cot_time(),
            "start": pytak.cot_time(),
            "stale": pytak.cot_time(3600),
        },
    )

    # Create point element
    point = ET.SubElement(
        event,
        "point",
        {
            "lat": f"{gps.latitude}",
            "lon": f"{gps.longitude}",
            "hae": "9999999.0",
            "ce": "9999999.0",
            "le": "9999999.0",
        },
    )

    # Create detail element
    detail = ET.SubElement(event, "detail")

    # Add contact element
    contact = ET.SubElement(detail, "contact", {"callsign": casualty_id})

    # Add link element
    link = ET.SubElement(
        detail,
        "link",
        {
            "type": "a-f-G-U-C-I",
            "uid": "S-1-5-21-942292099-3747883346-3641641706-1000",
            "parent_callsign": casualty_id,
            "relation": "p-p",
            "production_time": pytak.cot_time(),
        },
    )

    # Add archive and status elements
    ET.SubElement(detail, "archive")
    ET.SubElement(detail, "status", {"readiness": "false"})
    ET.SubElement(detail, "remarks")

    # Create _medevac_ element with nested zMistsMap and zMist
    medevac = ET.SubElement(
        detail,
        "_medevac_",
        {
            "title": casualty_id.upper(),
            "casevac": "false",
            "freq": "0.0",
            "equipment_none": "true",
            "security": "0",
            "hlz_marking": "3",
            "terrain_none": "true",
            "obstacles": "None",
            "medline_remarks": "",
            "zone_prot_selection": "0",
        },
    )

    zMistsMap = ET.SubElement(medevac, "zMistsMap")
    zMist = ET.SubElement(
        zMistsMap,
        "zMist",
        {
            "z": zap_num,
            "m": mechanism,
            "i": injury,
            "s": signs_symptoms,
            "t": treatments,
            "title": physician_name,
        },
    )

    # Add _flow-tags_ element
    flow_tags = ET.SubElement(
        detail,
        "_flow-tags_",
        {
            "TAK-Server-f6edbf55ccfa4af1b4cfa2d7f177ea67": f"chiron-tak_subscriber: {datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')}"
        },
    )

    # Convert to a string
    return ET.tostring(event, encoding="utf-8").decode("utf-8")

def create_chat_COT(uuid, callsign: str, message: str) -> str:

    # Create root event element
    event = ET.Element(
        "event",
        {
            "version": "2.0",
            "uid": f"GeoChat.{uuid}",
            "type": "b-t-f",
            "how": "h-g-i-g-o",
            "time": pytak.cot_time(),
            "start": pytak.cot_time(),
            "stale": pytak.cot_time(3600),
        }
    )

    # Create point element
    point = ET.SubElement(
        event,
        "point",
        {
            "lat": "0.0",
            "lon": "0.0",
            "hae": "9999999.0",
            "ce": "9999999.0",
            "le": "9999999.0",
        },
    )

    # Create detail element
    detail = ET.SubElement(event, "detail")

    # Create __chat element
    chat = ET.SubElement(
        detail,
        "__chat",
        {
            "id": "All Chat Rooms",
            "chatroom": "All Chat Rooms",
            "senderCallsign": callsign,
            "groupOwner": "false",
        }
    )

    # Add chatgrp element
    chatgrp = ET.SubElement(
        chat,
        "chatgrp",
        {
            "id": "All Chat Rooms",
            "uid0": uuid,
            "uid1": "All Chat Rooms",
        },
    )

    # Add link element
    link = ET.SubElement(
        detail,
        "link",
        {
            "uid": uuid,
            "type": "a-f-G-U-C-I",
            "relation": "p-p",
        },
    )

    # Add remarks element
    remarks = ET.SubElement(
        detail,
        "remarks",
        {
            "source": f"BAO.F.AIRLAB_CLI_MANAGER_{uuid}",
            "sourceID": uuid,
            "to": "All Chat Rooms",
            "time": datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%fZ"),
        }
    )
    remarks.text = message

    # Add _flow-tags_ element
    flow_tags = ET.SubElement(
        detail,
        "_flow-tags_",
        {
            "TAK-Server-f6edbf55ccfa4af1b4cfa2d7f177ea67": datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
        },
    )

    # Convert to a string
    return ET.tostring(event, encoding="utf-8").decode("utf-8")

from datetime import datetime, timedelta

def create_polygon_COT(
        uuid: str,
        callsign: str,
        gps_coordinates: List[Tuple[float, float]],
        fill_color: int = 16777215,
) -> bytes:
    """
    Creates a CoT message for a polygon using GPS coordinates.

    Args:
        gps_coordinates (list of tuples): List of (latitude, longitude) coordinates.
        uuid (str): Unique identifier for the CoT event.
        callsign (str): Callsign for the contact.
        fill_color (int, optional): Fill color value. Default is 16777215.
                    fill_color = (Red (0-255) << 16) + (Green (0-255) << 8) + Blue (0-255)


    Returns:
        bytes: CoT message as a UTF-8 encoded XML byte string.
    """
    if not gps_coordinates:
        raise ValueError("GPS coordinates list cannot be empty.")

    # Ensure the polygon is closed
    if gps_coordinates[0] != gps_coordinates[-1]:
        gps_coordinates.append(gps_coordinates[0])

    # Create the root element
    root = ET.Element("event", {
        "version": "2.0",
        "uid": uuid,
        "type": "u-d-f",
        "how": "h-e",
        "time": pytak.cot_time(),
        "start": pytak.cot_time(),
        "stale": pytak.cot_time(3600),
    })

    # Add the point element
    first_point = gps_coordinates[0]
    ET.SubElement(root, "point", {
        "lat": str(first_point[0]),
        "lon": str(first_point[1]),
        "hae": "9999999.0",
        "ce": "9999999.0",
        "le": "9999999.0",
    })

    # Add the detail element
    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "contact", {"callsign": callsign})
    ET.SubElement(detail, "strokeColor", {"value": "-1"})
    ET.SubElement(detail, "fillColor", {"value": str(fill_color)})
    ET.SubElement(detail, "remarks")
    ET.SubElement(detail, "height", {"value": "0.00"})
    ET.SubElement(detail, "height_unit", {"value": "4"})
    ET.SubElement(detail, "archive")

    # Add link elements for each GPS point
    for lat, lon in gps_coordinates:
        ET.SubElement(detail, "link", {"point": f"{lat},{lon}"})

    ET.SubElement(detail, "archive")

    # Generate the XML string
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)