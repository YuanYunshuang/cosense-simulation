import xml.etree.ElementTree as ET
from xml.dom import minidom
import random
import os
import csv
import shutil


def read_vtypes(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    vtypes_cls = {}
    vtypes_size = {}
    for vt in root.findall('vType'):
        vtypes_cls[vt.attrib['id']] = vt.attrib['vClass']
        vtypes_size[vt.attrib['id']] = [vt.attrib['length'],
                                        vt.attrib['width'],
                                        vt.attrib['height']]
    return vtypes_cls, vtypes_size


def write_bbox(data_path, vtypes_file, out_path):
    vtypes_cls, vtypes_size = read_vtypes(vtypes_file)
    # get all junctions
    junctions = os.listdir(data_path)
    for junc in junctions:
        info_file = os.path.join(data_path, junc, 'info.csv')
        with open(info_file, 'r') as fh:
            # first row is table header, data start from the second line
            for line in fh.readlines()[1:]:
                values = line.strip().split(',')
                frame = values.pop(0)
                junction = junc[1:].zfill(4)
                with open(os.path.join(out_path, junction + '_' + frame + '.txt'), 'w') as flbl:
                    for i in range(len(values))[::8]:
                        type_id = values[i]
                        id = values[i+1]
                        tf = values[i+2:i+8]
                        v_cls = vtypes_cls[type_id]
                        v_size = [float(s) for s in vtypes_size[type_id]]
                        ss = '{} '* 8 + '{:.2f} {:.2f} {:.2f}\n'
                        flbl.write(ss.format(id, v_cls, *tf, *v_size))

if __name__=="__main__":
    cur_dir = os.path.dirname(__file__)
    path = "/media/hdd/yuan/koko/data/simulation"
    file_vtypes = os.path.join(cur_dir, "../../data/carlavtypes.rou.xml")
    out_path = "/media/hdd/yuan/koko/data/synthdata/label/bbox"
    if os.path.exists(out_path):
        shutil.rmtree(out_path)
    os.makedirs(out_path)
    write_bbox(path, file_vtypes, out_path)



