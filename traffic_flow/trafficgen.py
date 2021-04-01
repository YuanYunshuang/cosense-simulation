import xml.etree.ElementTree as ET
from xml.dom import minidom
import random
import os


def prettify(elem):
    """
    Retrun a pretty printed XML string for the Element
    :param elem:
    :return:
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def read_net(net_filename):

    tree = ET.parse(net_filename)
    root = tree.getroot()
    # find junctions: ids, coordinates
    junctions = [j for j in root.findall('junction')
                 if '.' not in j.attrib['id'] and ':' not in j.attrib['id']]
    junc_coors_dic = {j.attrib['id']:[float(j.attrib['x']), float(j.attrib['y'])] for j in junctions}
    # find edges and the correspoding incoming and outgoing junctions
    incomings = {}
    outgoings = {}
    edges_dic = {}
    for j in junc_coors_dic.keys():
        incomings[j] = []
        outgoings[j] = []
    for edge in root.findall('edge'):
        if 'from' in edge.attrib:
            edges_dic[edge.attrib['id']] = [edge.attrib['from'], edge.attrib['to']]
            if '.' not in edge.attrib['from'] and ':' not in edge.attrib['from']:
                outgoings[edge.attrib['from']].append(edge.attrib['id'])
            if '.' not in edge.attrib['to'] and ':' not in edge.attrib['to']:
                incomings[edge.attrib['to']].append(edge.attrib['id'])
    # find all connections
    connections = {} # from_edge_id: list(to_edge_id)
    for c in root.findall('connection'):
        if connections.get(c.attrib['from'], None) is None:
            connections[c.attrib['from']] = []

        connections[c.attrib['from']].append(c.attrib['to'])

    return junc_coors_dic, incomings, outgoings, edges_dic, connections


# Generate possible route file and additional file for each junction
def gen_routes(junctions, incomings, outgoings, edges_dic, connections, net_filename):
    for j in junctions:
        dir = net_filename.split('/')[0] + '/' + j
        if not os.path.exists(dir):
            os.makedirs(dir)

        ## Additional file for rerouting ##
        filename = dir + '/rerouter.add.xml'
        addtionals = ET.Element('addtionals')
        edges = incomings[j] + outgoings[j]
        for i, e in enumerate(edges):
            rerouter = ET.SubElement(addtionals, 'rerouter', {
                'id': 'rerouter_' + str(i+1),
                'edges': e,
            })
            interval = ET.SubElement(rerouter, 'interval', {
                'begin': '0',
                'end': '1e9',
            })
            prob = '%.6f' % (1/(len(edges) - 1))
            for ee in edges:
                if not ee==e:
                    destProbReroute = ET.SubElement(interval, 'destProbReroute', {
                        'id': ee,
                        'probability': prob,
                    })
        with open(filename, mode='w') as fh:
            fh.write(prettify(addtionals))

        ## Route file ##
        routes = ET.Element('routes')
        routes.set('xmlns:xsi','http://www.w3.org/2001/XMLSchema-instance')
        routes.set('xsi:noNamespaceSchemaLocation','http://sumo.dlr.de/xsd/routes_file.xsd')
        for i, edge_in in enumerate(incomings[j]):
            for o, edge_out in enumerate(outgoings[j]):
                # remove route that has the same edge as departing and arriving edge
                # if root.find('.//edge[@id="%s"]' % incomings[j][i]).attrib['from']==root.find('.//edge[@id="%s"]' % outgoings[j][o]).attrib['to']:
                #     continue
                # route can not has the opposite direction of the same street as departing and arriving edge
                # if edges_dic[edge_in][0]==edges_dic[edge_out][1] and edges_dic[edge_out][0]==edges_dic[edge_in][1]:

                # check if there is a connection between two edge
                if not edge_out in connections[edge_in]:
                    continue
                # create a route departing from edge i and arriving at edge o
                id = j + '_' + incomings[j][i].split('.')[0] + '_' + outgoings[j][o].split('.')[0]
                edges = incomings[j][i] + ' ' + outgoings[j][o]
                route = ET.SubElement(routes, 'route', {
                    'id': id,
                    'color': 'yellow',
                    'edges': edges
                })

        # read carla vtypes
        tree = ET.parse("../data/carlavtypes.rou.xml")
        root = tree.getroot()
        vtypes = []
        for vt in root.findall('vType'):
            vtypes.append(vt.attrib['id'])
        # generate initial traffic
        n_vehicles = 100
        for i in range(n_vehicles):
            type = random.choice(vtypes)
            depart = "0.00" # str(i * 1.00)
            vroute = random.choice(routes.findall('route')).attrib['id']
            vehicle = ET.SubElement(routes, 'vehicle', {
                'id': str(i),
                'type': type,
                'depart': depart,
                'route': vroute
            })

        with open(dir + '/route.rou.xml', mode='w') as fh:
            fh.write(prettify(routes))

if __name__=="__main__":
    net_filename = 'town5/Town05.net.xml'
    junc_coors_filename = 'town5/junction_coordinates.json'
    junctions, incomings, outgoings, edges_dic, connections = read_net(net_filename)
    gen_routes(list(junctions.keys()), incomings, outgoings, edges_dic, connections, net_filename)
    import json
    with open(junc_coors_filename, 'w', encoding='utf-8') as f:
        json.dump(junctions, f, ensure_ascii=False, indent=4)

