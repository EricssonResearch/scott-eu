using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace PDDLPlanning
{
    public static class Graphviz
    {
        public const string LIB_GVC = @".\external\gvc.dll";
        public const string LIB_GRAPH = @".\external\cgraph.dll";
        public const int SUCCESS = 0;

        ///
        /// Creates a new Graphviz context.
        ///
        [DllImport(LIB_GVC, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr gvContext();

        ///
        /// Releases a context's resources.
        ///
        [DllImport(LIB_GVC, CallingConvention = CallingConvention.Cdecl)]
        public static extern int gvFreeContext(IntPtr gvc);

        ///
        /// Reads a graph from a string.
        ///
        [DllImport(LIB_GRAPH, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr agmemread(string data);

        ///
        /// Releases the resources used by a graph.
        ///
        [DllImport(LIB_GRAPH, CallingConvention = CallingConvention.Cdecl)]
        public static extern void agclose(IntPtr g);

        ///
        /// Applies a layout to a graph using the given engine.
        ///
        [DllImport(LIB_GVC, CallingConvention = CallingConvention.Cdecl)]
        public static extern int gvLayout(IntPtr gvc, IntPtr g, string engine);

        ///
        /// Releases the resources used by a layout.
        ///
        [DllImport(LIB_GVC, CallingConvention = CallingConvention.Cdecl)]
        public static extern int gvFreeLayout(IntPtr gvc, IntPtr g);

        ///
        /// Renders a graph to a file.
        ///
        [DllImport(LIB_GVC, CallingConvention = CallingConvention.Cdecl)]
        public static extern int gvRenderFilename(IntPtr gvc, IntPtr g,
              string format, string fileName);

        ///
        /// Renders a graph in memory.
        ///
        [DllImport(LIB_GVC, CallingConvention = CallingConvention.Cdecl)]
        public static extern int gvRenderData(IntPtr gvc, IntPtr g,
             string format, out IntPtr result, out int length);

        ///
        /// Release render resources.
        ///
        [DllImport(LIB_GVC, CallingConvention = CallingConvention.Cdecl)]
        public static extern int gvFreeRenderData(IntPtr result);


        public static Image RenderImage(string source, string format)
        {
            // Create a Graphviz context
            IntPtr gvc = gvContext();
            if (gvc == IntPtr.Zero)
                throw new Exception("Failed to create Graphviz context.");

            // Load the DOT data into a graph
            IntPtr g = agmemread(source);
            if (g == IntPtr.Zero)
                throw new Exception("Failed to create graph from source. Check for syntax errors.");

            // Apply a layout
            if (gvLayout(gvc, g, "dot") != SUCCESS)
                throw new Exception("Layout failed.");

            IntPtr result;
            int length;

            // Render the graph
            if (gvRenderData(gvc, g, format, out result, out length) != SUCCESS)
                throw new Exception("Render failed.");

            // Create an array to hold the rendered graph
            byte[] bytes = new byte[length];

            // Copy the image from the IntPtr
            Marshal.Copy(result, bytes, 0, length);

            // Free up the resources
            gvFreeLayout(gvc, g);
            agclose(g);
            gvFreeContext(gvc);
            gvFreeRenderData(result);
            using (MemoryStream stream = new MemoryStream(bytes))
            {
                return Image.FromStream(stream);
            }
        }
    }
}
