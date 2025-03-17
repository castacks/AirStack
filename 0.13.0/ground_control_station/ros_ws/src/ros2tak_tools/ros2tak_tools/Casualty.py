from distutils.core import setup

from sphinx.addnodes import index
from straps_msgs.msg import CasualtyMeta, Injury, Critical, Vitals
from xml.etree.ElementTree import Element, SubElement, tostring
import uuid
from datetime import datetime, timedelta
import pytak
from enum import Enum


"""
Enums to represent the different types of injuries and vitals.
"""

class TraumaType(Enum):
    TRAUMA_HEAD = Injury.TRAUMA_HEAD
    TRAUMA_TORSO = Injury.TRAUMA_TORSO
    TRAUMA_LOWER_EXT = Injury.TRAUMA_LOWER_EXT
    TRAUMA_UPPER_EXT = Injury.TRAUMA_UPPER_EXT
    ALERTNESS_OCULAR = Injury.ALERTNESS_OCULAR
    ALERTNESS_VERBAL = Injury.ALERTNESS_VERBAL
    ALERTNESS_MOTOR = Injury.ALERTNESS_MOTOR

class TraumaSeverity(Enum):
    TRAUMA_NORMAL = Injury.TRAUMA_NORMAL
    TRAUMA_WOUND = Injury.TRAUMA_WOUND
    TRAUMA_AMPUTATION = Injury.TRAUMA_AMPUTATION

class OcularAlertness(Enum):
    OCULAR_OPEN = Injury.OCULAR_OPEN
    OCULAR_CLOSED = Injury.OCULAR_CLOSED
    OCULAR_NOT_TESTABLE = Injury.OCULAR_NOT_TESTABLE

class AlertnessLevel(Enum):
    ALERTNESS_NORMAL = Injury.ALERTNESS_NORMAL
    ALERTNESS_ABNORMAL = Injury.ALERTNESS_ABNORMAL
    ALERTNESS_ABSENT = Injury.ALERTNESS_ABSENT
    ALERTNESS_NOT_TESTABLE = Injury.ALERTNESS_NOT_TESTABLE

class VitalType(Enum):
    HEART_RATE = Vitals.HEART_RATE
    RESPIRATORY_RATE = Vitals.RESPIRATORY_RATE

class ConditionType(Enum):
    SEVERE_HEMORRHAGE = Critical.SEVERE_HEMORRHAGE
    RESPIRATORY_DISTRESS = Critical.RESPIRATORY_DISTRESS

class ConditionStatus(Enum):
    ABSENT = Critical.ABSENT
    PRESENT = Critical.PRESENT

"""
Functions to validate the type and value of the injury.
"""
def is_valid_type_injury_value(trauma_type, value):
    """Validates that the value matches the type based on the rules."""
    if trauma_type in [TraumaType.TRAUMA_HEAD, TraumaType.TRAUMA_TORSO]:
        # TRAUMA_HEAD and TRAUMA_TORSO should have values TRAUMA_NORMAL or TRAUMA_WOUND
        return value in [TraumaSeverity.TRAUMA_NORMAL, TraumaSeverity.TRAUMA_WOUND]

    elif trauma_type in [TraumaType.TRAUMA_LOWER_EXT, TraumaType.TRAUMA_UPPER_EXT]:
        # TRAUMA_LOWER_EXT and TRAUMA_UPPER_EXT should have values TRAUMA_NORMAL, TRAUMA_WOUND, or TRAUMA_AMPUTATION
        return value in [TraumaSeverity.TRAUMA_NORMAL, TraumaSeverity.TRAUMA_WOUND, TraumaSeverity.TRAUMA_AMPUTATION]

    elif trauma_type == TraumaType.ALERTNESS_OCULAR:
        # ALERTNESS_OCULAR should have values OCULAR_OPEN, OCULAR_CLOSED, or OCULAR_NOT_TESTABLE
        return value in [OcularAlertness.OCULAR_OPEN, OcularAlertness.OCULAR_CLOSED, OcularAlertness.OCULAR_NOT_TESTABLE]

    elif trauma_type in [TraumaType.ALERTNESS_VERBAL, TraumaType.ALERTNESS_MOTOR]:
        # ALERTNESS_VERBAL and ALERTNESS_MOTOR should have values ALERTNESS_NORMAL, ALERTNESS_ABNORMAL, ALERTNESS_ABSENT, or ALERTNESS_NOT_TESTABLE
        return value in [AlertnessLevel.ALERTNESS_NORMAL, AlertnessLevel.ALERTNESS_ABNORMAL, AlertnessLevel.ALERTNESS_ABSENT, AlertnessLevel.ALERTNESS_NOT_TESTABLE]

    return False

# Function to create a unique casualty ID
def create_casualty_id(casualty_id:int):
    return f"casualty-{casualty_id}"

"""
Classes to represent a Casualty object with all the necessary information for triage.
"""

class GPSCOT:
    """
    GPS class to store the GPS coordinates of the casualty.
    """
    # Define the types
    status: bool
    latitude: float
    longitude: float
    altitude: float

    def __init__(self):
        self.status = False
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0

    def set_gps(self, latitude, longitude, altitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.status = True


class VitalsCOT:
    """
    Vitals class to store the vitals
    """
    # Define the class types
    vitals_name: str
    status: bool # True if diagnosis has been made, False otherwise
    system: str
    type: VitalType
    value: float
    time_ago: float
    confidence: float

    # initialize the class with default values
    def __init__(self, vitals_name="", system='', type_=None, value=None, time_ago=None, confidence=0.0):
        self.vitals_name = vitals_name
        self.status = False
        self.system = system
        self.type = type_
        self.value = value
        self.time_ago = time_ago
        self.confidence = confidence

    def set_vitals(self, system, type_, value, time_ago, confidence):
        # Ensuring that type is either HEART_RATE or RESPIRATORY_RATE
        if type_ not in [VitalType.HEART_RATE, VitalType.RESPIRATORY_RATE]:
            raise ValueError("Type must be either HEART_RATE (2) or RESPIRATORY_RATE (3)")

        self.system = system
        self.type = type_
        self.value = value
        self.time_ago = time_ago
        self.confidence = confidence
        self.status = True

    def __repr__(self):
        return f"{self.vitals_name}(Confidence={self.confidence*100:0.2f}%)"

class InjuryCOT:
    """
    Injury class to store the injury status
    """
    # Define the class types
    injury_name: str
    status: bool # True if diagnosis has been made, False otherwise
    system: str
    type: TraumaType
    value: TraumaSeverity
    confidence: float

    def __init__(self, injury_name="", system='', type_=None, value=None, confidence=0.0):
        self.injury_name = injury_name
        self.status = False
        self.system = system
        self.type_ = type_
        self.value = value
        self.confidence = confidence

    def set_status(self, system, type_, value, confidence):
        """Sets the status of the injury, with validation for type and value."""
        if not is_valid_type_injury_value(type_, value):
            raise ValueError(f"Invalid value for type {type_}: {value}")

        self.system = system
        self.type_ = type_
        self.value = value
        self.confidence = confidence
        self.status = True

    def __repr__(self):
        return f"{self.injury_name}(Confidence={self.confidence*100:0.2f}%)"


class CriticalCOT:
    """
    Critical class to store the critical condition status
    """
    # Define the class types
    critical_name: str
    status: bool # True if diagnosis has been made, False otherwise
    system: str
    type: ConditionType
    value: ConditionStatus
    confidence: float

    def __init__(self, critical_name="", system='', type_=None, value=None, confidence=0.0):
        self.critical_name = critical_name
        self.status = False
        self.system = system
        self.type = type_
        self.value = value
        self.confidence = confidence

    def set_status(self, system: str, type_: ConditionType, value: ConditionStatus, confidence: float):
        """Sets the status of the injury, with validation for type and value."""
        self.system = system
        self.type = type_
        self.value = value
        self.confidence = confidence
        self.status = True

    def __repr__(self):
        return f"{self.critical_name}(Confidence={self.confidence*100:0.2f}%)"


class CasualtyCOT:
    """
    Casualty class to store all the information of a casualty.
    """
    # Define the class types
    gps: GPSCOT
    stamp: str
    casualty_id: str
    # Critical conditions
    severe_hemorrhage: CriticalCOT
    respiratory_distress: CriticalCOT
    # Vitals
    heart_rate: VitalsCOT
    respiratory_rate: VitalsCOT
    # Injuries
    trauma_head: InjuryCOT
    trauma_torso: InjuryCOT
    trauma_lower_ext: InjuryCOT
    trauma_upper_ext: InjuryCOT
    # Alertness
    alertness_ocular: InjuryCOT
    alertness_verbal: InjuryCOT
    alertness_motor: InjuryCOT

    def __init__(self, casualty_id:int):
        self.stamp = pytak.cot_time()

        self.casualty_id = create_casualty_id(casualty_id)

        self.gps = GPSCOT()

        self.severe_hemorrhage = CriticalCOT("Severe Hemorrhage")
        self.respiratory_distress = CriticalCOT("Respiratory Distress")

        self.heart_rate = VitalsCOT("Heart Rate")
        self.respiratory_rate = VitalsCOT("Respiratory Rate")

        self.trauma_head = InjuryCOT("Trauma Head")
        self.trauma_torso = InjuryCOT("Trauma Torso")
        self.trauma_lower_ext = InjuryCOT("Trauma Lower Extremity")
        self.trauma_upper_ext = InjuryCOT("Trauma Upper Extremity")

        self.alertness_ocular = InjuryCOT("Alertness Ocular")
        self.alertness_verbal = InjuryCOT("Alertness Verbal")
        self.alertness_motor = InjuryCOT("Alertness Motor")

    # ZMIST Report Fields:
    # Z: Zap Number – A unique identifier for the casualty.
    # M: Mechanism of Injury – Describes how the injury occurred (e.g., explosion, gunshot wound).
    # I: Injuries Sustained – Specifies the injuries observed (e.g., right leg amputation).
    # S: Signs and Symptoms – Details vital signs and symptoms (e.g., massive hemorrhage, no radial pulse).
    # T: Treatments Rendered – Lists the medical interventions provided (e.g., tourniquet applied, pain medication administered).

    def get_zap_num(self):
        return f"{self.casualty_id}"

    def get_mechanism(self):
        return "Unknown"

    def get_injuries(self):
        """Returns the injuries sustained for preset status"""

        injuries = []
        if self.trauma_head.status:
            injuries.append(self.trauma_head)
        if self.trauma_torso.status:
            injuries.append(self.trauma_torso)
        if self.trauma_lower_ext.status:
            injuries.append(self.trauma_lower_ext)
        if self.trauma_upper_ext.status:
            injuries.append(self.trauma_upper_ext)
        return ", ".join(injuries)

    def get_signs_symptoms(self):
        """Returns the signs and symptoms for preset status"""

        signs = []
        if self.severe_hemorrhage.status:
            signs.append(self.severe_hemorrhage)
        if self.respiratory_distress.status:
            signs.append(self.respiratory_distress)
        if self.heart_rate.status:
            signs.append(self.heart_rate)
        if self.respiratory_rate.status:
            signs.append(self.respiratory_rate)
        return ", ".join(signs)

    def get_treatments(self):
        return "Unknown"

    def update_casualty_metadata(self, msg: CasualtyMeta):
        """Updates the casualty metadata with the message data."""

        # Update GPS coordinates
        if msg.valid_gps:
            self.gps.set_gps(msg.gps.latitude, msg.gps.longitude, msg.gps.altitude)

        # Update critical conditions
        if msg.valid_severe_hemorrhage:
            self.severe_hemorrhage.set_status(
                system="Circulatory",
                type_=ConditionType.SEVERE_HEMORRHAGE,
                value=ConditionStatus(msg.severe_hemorrhage.value),
                confidence=msg.severe_hemorrhage.confidence
            )
        if msg.valid_respiratory_distress:
            self.respiratory_distress.set_status(
                system="Respiratory",
                type_=ConditionType.RESPIRATORY_DISTRESS,
                value=ConditionStatus(msg.respiratory_distress.value),
                confidence=msg.respiratory_distress.confidence
            )

        # Update vitals
        if msg.valid_heart_rate:
            self.heart_rate.set_vitals(
                system="Cardiovascular",
                type_=VitalType.HEART_RATE,
                value=msg.heart_rate.value,
                time_ago=msg.heart_rate.time_ago,
                confidence=msg.heart_rate.confidence
            )
        if msg.valid_respiratory_rate:
            self.respiratory_rate.set_vitals(
                system="Respiratory",
                type_=VitalType.RESPIRATORY_RATE,
                value=msg.respiratory_rate.value,
                time_ago=msg.respiratory_rate.time_ago,
                confidence=msg.respiratory_rate.confidence
            )

        # Update injuries
        if msg.valid_trauma_head:
            self.trauma_head.set_status(
                system="Head",
                type_=TraumaType.TRAUMA_HEAD,
                value=TraumaSeverity(msg.trauma_head.value),
                confidence=msg.trauma_head.confidence
            )
        if msg.valid_trauma_torso:
            self.trauma_torso.set_status(
                system="Torso",
                type_=TraumaType.TRAUMA_TORSO,
                value=TraumaSeverity(msg.trauma_torso.value),
                confidence=msg.trauma_torso.confidence
            )
        if msg.valid_trauma_lower_ext:
            self.trauma_lower_ext.set_status(
                system="Lower Extremity",
                type_=TraumaType.TRAUMA_LOWER_EXT,
                value=TraumaSeverity(msg.trauma_lower_ext.value),
                confidence=msg.trauma_lower_ext.confidence
            )
        if msg.valid_trauma_upper_ext:
            self.trauma_upper_ext.set_status(
                system="Upper Extremity",
                type_=TraumaType.TRAUMA_UPPER_EXT,
                value=TraumaSeverity(msg.trauma_upper_ext.value),
                confidence=msg.trauma_upper_ext.confidence
            )

        # Update alertness levels
        if msg.valid_alertness_ocular:
            self.alertness_ocular.set_status(
                system="Neurological",
                type_=TraumaType.ALERTNESS_OCULAR,
                value=OcularAlertness(msg.alertness_ocular.value),
                confidence=msg.alertness_ocular.confidence
            )
        if msg.valid_alertness_verbal:
            self.alertness_verbal.set_status(
                system="Neurological",
                type_=TraumaType.ALERTNESS_VERBAL,
                value=AlertnessLevel(msg.alertness_verbal.value),
                confidence=msg.alertness_verbal.confidence
            )
        if msg.valid_alertness_motor:
            self.alertness_motor.set_status(
                system="Neurological",
                type_=TraumaType.ALERTNESS_MOTOR,
                value=AlertnessLevel(msg.alertness_motor.value),
                confidence=msg.alertness_motor.confidence
            )

    def generate_cot_event(self):
        # Create root event element
        event = Element('event', {
            'version': "2.0",
            'uid': str(uuid.uuid4()),  # Generate a unique UID
            'type': "b-r-f-h-c",
            'how': "h-g-i-g-o",
            'time': self.stamp,
            'start': self.stamp,
            'stale': pytak.cot_time(2400)
        })

        # Create point element
        point = SubElement(event, 'point', {
            'lat': f"{self.gps.latitude}",
            'lon': f"{self.gps.longitude}",
            'hae': "9999999.0",
            'ce': "9999999.0",
            'le': "9999999.0"
        })

        # Create detail element
        detail = SubElement(event, 'detail')

        # Add contact element
        contact = SubElement(detail, 'contact', {
            'callsign': self.casualty_id
        })

        # Add link element
        link = SubElement(detail, 'link', {
            'type': "a-f-G-U-C-I",
            'uid': "S-1-5-21-942292099-3747883346-3641641706-1000",
            'parent_callsign': self.casualty_id,
            'relation': "p-p",
            'production_time': self.stamp
        })

        # Add archive and status elements
        SubElement(detail, 'archive')
        SubElement(detail, 'status', {'readiness': "false"})
        SubElement(detail, 'remarks')

        # Create _medevac_ element with nested zMistsMap and zMist
        medevac = SubElement(detail, '_medevac_', {
            'title': "MED.12.201008",
            'casevac': "false",
            'freq': "0.0",
            'equipment_none': "true",
            'security': "0",
            'hlz_marking': "3",
            'terrain_none': "true",
            'obstacles': "None",
            'medline_remarks': "",
            'zone_prot_selection': "0"
        })

        zMistsMap = SubElement(medevac, 'zMistsMap')
        zMist = SubElement(zMistsMap, 'zMist', {
            'z': self.get_zap_num(),
            'm': self.get_mechanism(),
            'i': self.get_injuries(),
            's': self.get_signs_symptoms(),
            't': self.get_treatments(),
            'title': "ZMIST1"
        })

        # Add _flow-tags_ element
        flow_tags = SubElement(detail, '_flow-tags_', {
            'TAK-Server-f6edbf55ccfa4af1b4cfa2d7f177ea67': f"chiron-tak_subscriber: {datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')}"
        })

        # Convert to a string
        return tostring(event, encoding='utf-8').decode('utf-8')




