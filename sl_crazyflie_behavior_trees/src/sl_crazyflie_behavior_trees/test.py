#!/usr/bin/env python
from BehaviorTree import BehaviorTree, AIIO

if __name__ == '__main__':
    bt = BehaviorTree.load_from_file("/home/broecker/src/collvoid_ea_results/dw1000_face_true_default_param/Genomes/Genome_55_t.xml")
    in_out = AIIO()
    in_out.input = [-5.0]
    in_out.output = [0, 0, 0, 0]
    bt.trigger(in_out)
    print in_out.output
