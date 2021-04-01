import xml.etree.ElementTree as ET
from xml.dom import minidom
import random


def prettify(elem):
    """
    Retrun a pretty printed XML string for the Element
    :param elem:
    :return:
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

root_path = '.'

tree = ET.parse(root_path + "/data/net/Town05.net.xml")
root = tree.getroot()

junctions = [j.attrib['id'] for j in root.findall('junction')]
incomings = {}
outgoings = {}
for j in junctions:
    incomings[j] = []
    outgoings[j] = []
for edge in root.findall('edge'):
    if 'from' in edge.attrib:
        outgoings[edge.attrib['from']].append(edge.attrib['id'])
        incomings[edge.attrib['to']].append(edge.attrib['id'])

# Generate possible route for each junction
routes = ET.Element('routes')
routes.set('xmlns:xsi','http://www.w3.org/2001/XMLSchema-instance')
routes.set('xsi:noNamespaceSchemaLocation','http://sumo.dlr.de/xsd/routes_file.xsd')
#for j in junctions:
j = '421'
for i in range(len(incomings[j])):
    for o in range(len(outgoings[j])):
        if root.find('.//edge[@id="%s"]' % incomings[j][i]).attrib['from']==root.find('.//edge[@id="%s"]' % outgoings[j][o]).attrib['to']:
            continue
        id = j + '_' + incomings[j][i].split('.')[0] + '_' + outgoings[j][o].split('.')[0]
        edges = incomings[j][i] + ' ' + outgoings[j][o]
        route = ET.SubElement(routes, 'route', {
            'id': id,
            'color': 'yellow',
            'edges': edges
        })

# read carla vtypes
# /home/ophelia/carla/Co-Simulation/Sumo/examples/ophelia/carlavtypes.rou.xml
tree = ET.parse(root_path + "/data/carlavtypes.rou.xml")
root = tree.getroot()
vtypes = []
for vt in root.findall('vType'):
    vtypes.append(vt.attrib['id'])

n_vehicles = 50
# file:///home/ophelia/carla/Co-Simulation/Sumo/examples/ophelia/rou/Town05_421_.rou.xml
# example for junction 421
for i in range(n_vehicles):
    type = random.choice(vtypes)
    depart = str(i * 1.00)
    vroute = random.choice(routes.findall('route')).attrib['id']
    vehicle = ET.SubElement(routes, 'vehicle', {
        'id': str(i),
        'type': type,
        'depart': depart,
        'route': vroute
    })

with open(root_path + '/data/rou/Town05_421.rou.xml', mode='w') as fh:
    fh.write(prettify(routes))