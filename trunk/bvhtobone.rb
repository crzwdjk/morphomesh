#!/usr/bin/ruby

# parse a .bvh file and output a pseudo-obj
# syntax
# rule a { .* \{ .* OFFSET <num> <num> <num> [ JOINT .*  <a> ]*\} }

# File -> [ vertices, edges ]
def parse_bvh(file)
  nodecount = 0
  stack = []
  vertices = []
  edges = []

  file.each do |line|
    words = line.split
    case words[0]
    when "OFFSET"
      vertices.push([words[1].to_f, words[2].to_f, words[3].to_f])
    when "{"
      stack.push(nodecount)
      nodecount += 1
    when "}"
      if stack.size > 1
        edges.push([stack[-1], stack[-2]])
        stack.pop
      end
    end
  end
  [ vertices, edges ]
end

def print_bone(vertices, edges)
  for v in vertices
    puts "v #{v[0]} #{v[1]} #{v[2]}"
  end
  for e in edges
    puts "e #{e[0]} #{e[1]}"
  end
end

print_bone(*parse_bvh($stdin))

__END__

require 'test/unit'

class TC_ParseTest < Test::Unit::TestCase
  def split_bvh(str); parse_bvh(str.split("\n")); end
  def test_simple
    assert_equal(split_bvh("{\nOFFSET 0 1.0 .1\n}"), [[[0.0, 1.0, 0.1]], []])
    bvh1 = "{\nOFFSET 1 2 3\n{\n  OFFSET 2 3 4\n}\n}\n"
    assert_equal(split_bvh(bvh1), [[[1, 2, 3], [2, 3, 4]], [[1, 0]]])
    bvh2 = "{\nOFFSET 1 2 3\n{\nOFFSET 2 3 4\n}\nFISH\n{\nOFFSET 4 5 6\n}\n}"
    assert_equal(split_bvh(bvh2), [[[1, 2, 3], [2, 3, 4], [4, 5, 6]], [[1, 0], [2, 0]]])
    bvh3 = "{\nOFFSET 1 2 3\n{\nOFFSET 4 5 6\n{\nOFFSET 7 8 9\n}\n}\n}"
    assert_equal(split_bvh(bvh3), [[[1, 2, 3], [4, 5, 6], [7, 8, 9]], [[2, 1], [1, 0]]])
  end
end
