using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace K2Collisions
{
    public class K2CollisionsInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "K2Collisions";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("f95f267b-f340-4e81-925e-31c2e832d718");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "KADK";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
