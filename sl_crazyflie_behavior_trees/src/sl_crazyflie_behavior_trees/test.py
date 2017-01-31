#!/usr/bin/env python
from BehaviorTree import BehaviorTree, AIIO

if __name__ == '__main__':
    bt = BehaviorTree.load_from_file("/home/broecker/collvoid_ea_results/manual_genomes/Genome_manual_bluetooth.xml")
    in_out = AIIO()
    in_out.input = [0.0]
    bt.trigger(in_out)
    print in_out.output
    in_out.input = [0.0]
    bt.trigger(in_out)
    print in_out.output
    in_out.input = [-0.8]
    bt.trigger(in_out)
    print in_out.output
    in_out.input = [-0.8]
    bt.trigger(in_out)
    print in_out.output
    # in_out.input = [-0.65]
    # bt.trigger(in_out)
    # print in_out.output
    # in_out.input = [-0.80]
    # bt.trigger(in_out)
    # print in_out.output
    # in_out.input = [-0.80]
    # bt.trigger(in_out)
    # print in_out.output
    # in_out.input = [0.0]
    # bt.trigger(in_out)
    # print in_out.output
